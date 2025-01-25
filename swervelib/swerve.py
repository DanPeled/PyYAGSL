from enum import Enum
from typing import Callable, Dict, Final, List, Optional, Tuple, Union
import numpy as np
from wpimath.trajectory import Trajectory
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import (
    Pose2d,
    Rotation2d,
    Rotation3d,
    Transform2d,
    Translation2d,
    Translation3d,
)
from wpimath.system.plant import DCMotor
from wpimath.units import (
    inchesToMeters,
    meters_per_second,
    degrees_per_second,
    degrees,
    metersToInches,
    radiansToRotations,
    rotationsToDegrees,
    volts,
    radians,
    seconds,
    milliseconds,
    newtons,
    meters,
)
from wpimath.kinematics import (
    SwerveDrive4Kinematics,
    SwerveModulePosition,
    SwerveModuleState,
    ChassisSpeeds,
)
import hal as hal
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpilib import Alert, Field2d, Notifier, RobotBase, SmartDashboard
from wpimath.controller import PIDController, SimpleMotorFeedforwardMeters
from ntcore import BooleanPublisher, DoublePublisher, NetworkTableInstance
from swervelib.encoders import SwerveAbsoluteEncoder
from swervelib.math import SwerveMath
from swervelib.motors import SwerveMotor
from swervelib.parser.cache import Cache
from swervelib.parser.moduleConfig import SwerveModuleConfiguration
from swervelib.parser.pidf import PIDFConfig
from swervelib.parser.swerve import (
    SwerveControllerConfiguration,
    SwerveDriveConfiguration,
)
import threading as thrd
from swervelib.simDevices import SwerveIMUSimulation, SwerveModuleSimulation
from swervelib.telemetry import SwerveDriveTelemetry, TelemetryVerbosity
from swervelib.imu import SwerveIMU

FloatSupplier = Callable[[], float]
BooleanSupplier = Callable[[], bool]


class SwerveModule:
    def __init__(
        self, moduleNumber: int, moduleConfiguration: SwerveModuleConfiguration
    ) -> None:
        self.__maxDriveVelocity: meters_per_second
        self.__maxAngularVelocity: degrees_per_second
        self.__antiJitterEnabled: bool = True
        self.__synchronizeEncoderQueued: bool = False
        self.__synchronizeEncoderEnabled: bool = False
        self.__synchronizeEncoderDeadband: degrees = 3
        self.__simModule: SwerveModuleSimulation
        self.moduleNumber: Final[int] = moduleNumber
        self.configuration: Final[SwerveModuleConfiguration] = moduleConfiguration
        self.__angleOffset: float = self.configuration.angleOffset

        self.__angleMotor: SwerveMotor = self.configuration.angleMotor
        self.__driveMotor: SwerveMotor = self.configuration.driveMotor
        self.__angleMotor.factoryDefaults()
        self.__driveMotor.factoryDefaults()

        self.__driveMotorFeedforward: SimpleMotorFeedforwardMeters = (
            self.getDefaultFeedforward()
        )

        self.__angleMotor.setVoltageCompensation(
            self.configuration.physicalCharacteristics.optimalVoltage
        )
        self.__driveMotor.setVoltageCompensation(
            self.configuration.physicalCharacteristics.optimalVoltage
        )
        self.__angleMotor.setCurrentLimit(
            self.configuration.physicalCharacteristics.angleMotorCurrentLimit
        )
        self.__driveMotor.setCurrentLimit(
            self.configuration.physicalCharacteristics.driveMotorCurrentLimit
        )
        self.__angleMotor.setLoopRampRate(
            self.configuration.physicalCharacteristics.angleMotorRampRate
        )
        self.__driveMotor.setLoopRampRate(
            self.configuration.physicalCharacteristics.driveMotorRampRate
        )

        self.__absoluteEncoder: Final[SwerveAbsoluteEncoder] = (
            self.configuration.absoluteEncoder
        )

        if self.__absoluteEncoder is not None:
            self.__absoluteEncoder.factoryDefault()
            self.__absoluteEncoder.configure(self.configuration.absoluteEncoderInverted)

        if SwerveDriveTelemetry.isSimulation:
            self.__simModule = SwerveModuleSimulation()

        self.absolutePositionCache: Final[Cache[degrees]] = Cache(
            self.getRawAbsolutePosition, 20
        )

        if not self.__angleMotor.isAttachedAbsoluteEncoder():
            self.__angleMotor.configureIntegratedEncoder(
                self.configuration.conversionFactors.angle.factor
            )
        self.__angleMotor.configurePIDF(self.configuration.anglePIDF)
        self.__angleMotor.configurePIDWrapping(0, 360)
        self.__angleMotor.setInverted(self.configuration.angleMotorInverted)
        self.__angleMotor.setMotorBrake(False)

        if self.__absoluteEncoder is not None:
            self.__angleMotor.setPosition(self.getAbsolutePosition())

        self.__driveMotor.configureIntegratedEncoder(
            self.configuration.conversionFactors.drive.factor
        )
        self.__driveMotor.configurePIDF(self.configuration.velocityPIDF)
        self.__driveMotor.setInverted(self.configuration.driveMotorInverted)
        self.__driveMotor.setMotorBrake(True)

        self.__driveMotor.burnFlash()
        self.__angleMotor.burnFlash()

        self.drivePositionCache: Final[Cache[float]] = Cache(
            self.__driveMotor.getPosition, 20
        )
        self.driveVelocityCache: Final[Cache[meters_per_second]] = Cache(
            self.__driveMotor.getVelocity, 20
        )

        self.driveVelocityCache.update()
        self.drivePositionCache.update()
        self.absolutePositionCache.update()

        self.__lastState: SwerveModuleState = self.getState()

        self.__noEncoderWarning: Final[Alert] = Alert(
            "Motors",
            f"There is no Absolute Encoder on module #{self.moduleNumber}",
            Alert.AlertType.kWarning,
        )

        self.__encoderOffsetWarning: Final[Alert] = Alert(
            "Motors",
            f"Pushing the Absolute Encoder offset to the encoder failed on module #{self.moduleNumber}",
            Alert.AlertType.kWarning,
        )

        self.__rawAbsoluteAnglePublisher: Final[DoublePublisher] = (
            NetworkTableInstance.getDefault()
            .getTable("SmartDashboard")
            .getDoubleTopic(
                f"swerve/modules/{self.configuration.name}/Raw Absolute Encoder"
            )
            .publish()
        )
        self.__adjAbsoluteAnglePublisher: Final[DoublePublisher] = (
            NetworkTableInstance.getDefault()
            .getTable("SmartDashboard")
            .getDoubleTopic(
                f"swerve/modules/{self.configuration.name}/Adjusted Absolute Encoder"
            )
            .publish()
        )
        self.__absoluteEncoderIssuePublisher: Final[BooleanPublisher] = (
            NetworkTableInstance.getDefault()
            .getTable("SmartDashboard")
            .getBooleanTopic(
                f"swerve/modules/{self.configuration.name}/Absolute Encoder Read Issue"
            )
            .publish()
        )
        self.__rawAnglePublisher: Final[DoublePublisher] = (
            NetworkTableInstance.getDefault()
            .getTable("SmartDashboard")
            .getDoubleTopic(
                f"swerve/modules/{self.configuration.name}/Raw Angle Encoder"
            )
            .publish()
        )
        self.__rawDriveEncoderPublisher: Final[DoublePublisher] = (
            NetworkTableInstance.getDefault()
            .getTable("SmartDashboard")
            .getDoubleTopic(
                f"swerve/modules/{self.configuration.name}/Raw Drive Encoder"
            )
            .publish()
        )
        self.__rawDriveVelocityPublisher: Final[DoublePublisher] = (
            NetworkTableInstance.getDefault()
            .getTable("SmartDashboard")
            .getDoubleTopic(
                f"swerve/modules/{self.configuration.name}/Raw Drive Velocity"
            )
            .publish()
        )
        self.__speedsSetpointPublisher: Final[DoublePublisher] = (
            NetworkTableInstance.getDefault()
            .getTable("SmartDashboard")
            .getDoubleTopic(f"swerve/modules/{self.configuration.name}/Speed Setpoint")
            .publish()
        )
        self.__angleSetpointPublisher: Final[DoublePublisher] = (
            NetworkTableInstance.getDefault()
            .getTable("SmartDashboard")
            .getDoubleTopic(f"swerve/modules/{self.configuration.name}/Angle Setpoint")
            .publish()
        )

    def getPosition(self) -> SwerveModulePosition:
        position: float
        azimuth: Rotation2d
        if not SwerveDriveTelemetry.isSimulation:
            position = self.drivePositionCache.getValue()
            azimuth = Rotation2d.fromDegrees(self.getAbsolutePosition())
        else:
            return self.__simModule.getPosition()

        return SwerveModulePosition(position, azimuth)

    def getRawAbsolutePosition(self) -> degrees:
        if SwerveDriveTelemetry.isSimulation:
            absolutePosition: Rotation2d = self.__simModule.getState().angle
            return absolutePosition.degrees()

        angle: float
        if self.__absoluteEncoder is not None:
            angle = self.__absoluteEncoder.getAbsolutePosition() - self.__angleOffset
            if self.__absoluteEncoder.readingError:
                angle = self.getRelativePosition()

        else:
            angle = self.getRelativePosition()

        angle %= 360
        if angle < 0.0:
            angle += 360
        return angle

    def getRelativePosition(self) -> degrees:
        return self.__angleMotor.getPosition()

    def getAbsolutePosition(self) -> degrees:
        return self.absolutePositionCache.getValue()

    def getDefaultFeedforward(self) -> SimpleMotorFeedforwardMeters:
        nominalVoltage: volts = self.__driveMotor.getSimMotor().nominalVoltage
        maxDriveSpeed: meters_per_second = self.getMaxVelocity()
        return SwerveMath.createDriveFeedForward(
            nominalVoltage,
            maxDriveSpeed,
            self.configuration.physicalCharacteristics.wheelGripCoefficientOfFriction,
        )

    def getMaxVelocity(self) -> meters_per_second:
        self.getMaxDriveVelocity()
        return self.__maxDriveVelocity

    def getMaxDriveVelocity(self) -> meters_per_second:
        if self.__maxDriveVelocity is None:
            self.__maxDriveVelocity = inchesToMeters(
                (
                    self.__driveMotor.getSimMotor().freeSpeed
                    / self.configuration.conversionFactors.drive.gearRatio
                )
                * self.configuration.conversionFactors.drive.diameter
                / 2
            )
        return self.__maxDriveVelocity

    def getState(self) -> SwerveModuleState:
        velocity: meters_per_second
        azimuth: Rotation2d

        if not SwerveDriveTelemetry.isSimulation:
            velocity = self.driveVelocityCache.getValue()
            azimuth = Rotation2d.fromDegrees(self.getAbsolutePosition())
        else:
            return self.__simModule.getState()
        return SwerveModuleState(velocity, azimuth)

    def setAngleMotorVoltageCompensation(self, optimalVoltage: volts) -> None:
        self.__angleMotor.setVoltageCompensation(optimalVoltage)

    def setDriveMotorVoltageCompensation(self, optimalVoltage: volts) -> None:
        self.__driveMotor.setVoltageCompensation(optimalVoltage)

    def queueSynchronizeEncoders(self) -> None:
        if self.__absoluteEncoder is not None and self.__synchronizeEncoderEnabled:
            self.__synchronizeEncoderQueued = True

    def setEncoderAutoSynchronize(
        self, enabled: bool, deadband: Optional[degrees] = None
    ) -> None:
        self.__synchronizeEncoderEnabled = enabled
        if deadband is not None:
            self.__synchronizeEncoderDeadband = deadband

    def setAntiJitter(self, antiJitter: bool) -> None:
        self.__antiJitterEnabled = antiJitter

        if antiJitter:
            self.pushOffsetsToEncoders()
        else:
            self.restoreInternalOffset()

    def setFeedForward(self, driveFF: SimpleMotorFeedforwardMeters) -> None:
        self.__driveMotorFeedforward = driveFF

    def getDrivePIDF(self) -> PIDFConfig:
        return self.configuration.velocityPIDF

    def setDrivePIDF(self, config: PIDFConfig) -> None:
        self.configuration.velocityPIDF = config
        self.__driveMotor.configurePIDF(config)

    def getAnglePIDF(self) -> PIDFConfig:
        return self.configuration.anglePIDF

    def setAnglePIDF(self, config: PIDFConfig) -> None:
        self.configuration.anglePIDF = config
        self.__angleMotor.configurePIDF(config)

    def setDesiredState(
        self,
        desiredState: SwerveModuleState,
        isOpenLoop: bool,
        force: bool,
    ):
        desiredState.optimize(Rotation2d.fromDegrees(self.getAbsolutePosition()))
        if not force and self.__antiJitterEnabled:
            SwerveMath.antiJitter(
                desiredState, self.__lastState, min(self.__maxDriveVelocity, 4)
            )

        nextVelocity: meters_per_second = (
            self.getCosineCompensatedVelocity(desiredState)
            if self.configuration.useCosineCompensator
            else desiredState.speed
        )
        curVelocity: meters_per_second = self.__lastState.speed
        desiredState.speed = nextVelocity
        self.applyDesiredState(
            desiredState,
            isOpenLoop,
            self.__driveMotorFeedforward.calculate(curVelocity, nextVelocity),
        )

    def applyDesiredState(
        self,
        desiredState: SwerveModuleState,
        isOpenLoop: bool,
        driveFeedforwardVoltage: volts,
    ):
        if isOpenLoop:
            percent_output = desiredState.speed / self.__maxDriveVelocity
            self.__driveMotor.setVoltage(percent_output * 12)
        else:
            self.__driveMotor.setReference(desiredState.speed, driveFeedforwardVoltage)

        # Prevent module rotation if angle is the same as the previous angle.
        # Synchronize encoders if queued and send in the current position as the value from the absolute encoder.
        if (
            self.__absoluteEncoder is not None
            and self.__synchronizeEncoderQueued
            and self.__synchronizeEncoderEnabled
        ):
            absoluteEncoderPosition = self.getAbsolutePosition()
            if (
                abs(self.__angleMotor.getPosition() - absoluteEncoderPosition)
                >= self.__synchronizeEncoderDeadband
            ):
                self.__angleMotor.setPosition(absoluteEncoderPosition)
            self.__angleMotor.setReference(
                desiredState.angle.degrees(), 0, int(absoluteEncoderPosition)
            )
            self.__synchronizeEncoderQueued = False
        else:
            self.__angleMotor.setReference(desiredState.angle.degrees(), 0)

        self.__lastState = desiredState

        if SwerveDriveTelemetry.isSimulation:
            self.__simModule.updateStateAndPosition(desiredState)

        # TODO: Change and move to SwerveDriveTelemetry
        if SwerveDriveTelemetry.verbosity.value >= TelemetryVerbosity.INFO.value:
            SwerveDriveTelemetry.desiredStatesObj[self.moduleNumber] = desiredState

        if SwerveDriveTelemetry.verbosity == TelemetryVerbosity.HIGH:
            self.__speedsSetpointPublisher.set(desiredState.speed)
            self.__angleSetpointPublisher.set(desiredState.angle.degrees())

        if self.moduleNumber == SwerveDriveTelemetry.moduleCount - 1:
            SwerveDriveTelemetry.endCtrlCycle()

    def setAngle(self, angle: degrees) -> None:
        self.__angleMotor.setReference(angle, 0)
        self.__lastState.angle = Rotation2d.fromDegrees(angle)

    def setMotorBrake(self, brake: bool) -> None:
        self.__driveMotor.setMotorBrake(brake)

    def setAngleMotorConversionFactor(self, conversionFactor: float) -> None:
        self.__angleMotor.configureIntegratedEncoder(conversionFactor)

    def setDriveMotorConversionFactor(self, conversionFactor: float) -> None:
        self.__driveMotor.configureIntegratedEncoder(conversionFactor)

    def getAngleMotor(self) -> SwerveMotor:
        return self.__angleMotor

    def getDriveMotor(self) -> SwerveMotor:
        return self.__driveMotor

    def getAbsoluteEncoder(self) -> SwerveAbsoluteEncoder:
        return self.__absoluteEncoder

    def getConfiguration(self) -> SwerveModuleConfiguration:
        return self.configuration

    def pushOffsetsToEncoders(self) -> None:
        if (
            self.__absoluteEncoder is not None
            and self.__angleOffset == self.configuration.angleOffset
        ):
            ...  # TODO implement motors
            # if (
            #     self.__angleMotor is SparkMaxSwerve
            #     or self.__angleMotor is SparkMaxBrushedMotorSwerve
            # ):
            #     if self.__absoluteEncoder is SparkMaxEncoderSwerve:
            #         self.__angleMotor.setAbsoluteEncoder(self.__absoluteEncoder)
            #         if self.__absoluteEncoder.setAbsoluteEncoderOffset(
            #             self.__angleOffset
            #         ):
            #             self.__angleOffset = 0.0
            #         else:
            #             self.__angleMotor.setAbsoluteEncoder(None)
            #             self.__encoderOffsetWarning.set(True)
            #
        else:
            self.__noEncoderWarning.set(True)

    def restoreInternalOffset(self) -> None:
        self.__angleMotor.setAbsoluteEncoder(None)
        self.__absoluteEncoder.setAbsoluteEncoderOffset(0)
        self.__angleOffset = self.configuration.angleOffset

    def getAbsoluteEncoderReadIssue(self) -> bool:
        if self.__absoluteEncoder is None:
            return True
        return self.__absoluteEncoder.readingError

    def getMaxAngularVelocity(self) -> degrees_per_second:
        if self.__maxAngularVelocity is None:
            self.__maxAngularVelocity = radiansToRotations(
                self.__angleMotor.getSimMotor().freeSpeed
                * self.configuration.conversionFactors.angle.gearRatio
            )

        return rotationsToDegrees(self.__maxAngularVelocity)

    def updateTelemetry(self) -> None:
        if self.__absoluteEncoder is not None:
            self.__rawAbsoluteAnglePublisher.set(self.getAbsolutePosition())

        if (
            SwerveDriveTelemetry.isSimulation
            and SwerveDriveTelemetry.verbosity == TelemetryVerbosity.HIGH
        ):
            pos: SwerveModulePosition = self.__simModule.getPosition()
            state: SwerveModuleState = self.__simModule.getState()
            self.__rawAnglePublisher.set(pos.angle.degrees())
            self.__rawDriveEncoderPublisher.set(pos.distance)
            self.__rawDriveVelocityPublisher.set(state.speed)

            # For code coverage
            self.__angleMotor.getPosition()
            self.drivePositionCache.getValue()
            self.driveVelocityCache.getValue()
        else:
            self.__rawAnglePublisher.set(self.__angleMotor.getPosition())
            self.__rawDriveEncoderPublisher.set(self.drivePositionCache.getValue())
            self.__rawDriveVelocityPublisher.set(self.driveVelocityCache.getValue())

        self.__adjAbsoluteAnglePublisher.set(self.getAbsolutePosition())
        self.__rawDriveVelocityPublisher.set(self.getAbsoluteEncoderReadIssue())

    def invalidateCache(self) -> None:
        self.absolutePositionCache.update()
        self.drivePositionCache.update()
        self.driveVelocityCache.update()

    def getSimModule(self) -> SwerveModuleSimulation:
        return self.__simModule

    def getCosineCompensatedVelocity(
        self, desiredState: SwerveModuleState
    ) -> meters_per_second:
        cosineScalar: float = 1.0
        # Taken from the CTRE SwerveModule class.
        # https://api.ctr-electronics.com/phoenix6/release/java/src-html/com/ctre/phoenix6/mechanisms/swerve/SwerveModule.html#line.46
        # From FRC 900's whitepaper, we add a cosine compensator to the applied drive velocity
        # To reduce the "skew" that occurs when changing direction
        # If error is close to 0 rotations, we're already there, so apply full power
        # If the error is close to 0.25 rotations, then we're 90 degrees, so movement doesn't help us at all
        cosineScalar = (
            Rotation2d.fromDegrees(desiredState.angle.degrees())
            - Rotation2d.fromDegrees(self.getAbsolutePosition())
        ).degrees()

        if cosineScalar < 0.0:
            cosineScalar = 1.0

        return desiredState.speed * cosineScalar


class SwerveController:
    def __init__(self, cfg: SwerveControllerConfiguration):
        self.config: Final[SwerveControllerConfiguration] = cfg
        self.thetaController: Final[PIDController] = (
            cfg.headingPIDF.createPIDController()
        )
        self.lastAngleScalar: float = 0
        self.xLimiter: Optional[SlewRateLimiter] = None
        self.yLimiter: Optional[SlewRateLimiter] = None
        self.angleLimiter: Optional[SlewRateLimiter] = None

    def getTranslation2d(self, speeds: ChassisSpeeds) -> Translation2d:
        return Translation2d(speeds.vx, speeds.vy)

    def addSlewRateLimiters(
        self, x: SlewRateLimiter, y: SlewRateLimiter, angle: SlewRateLimiter
    ) -> None:
        self.xLimiter = x
        self.yLimiter = y
        self.angleLimiter = angle

    def withinHypotDeadband(self, x: float, y: float) -> bool:
        return np.hypot(x, y) < self.config.angleJoystickRadiusDeadband

    def getTargetSpeedsWithAngle(
        self,
        xInput: float,
        yInput: float,
        angle: float,
        currentHeadingAngle: radians,
        maxSpeed: meters_per_second,
    ) -> ChassisSpeeds:
        x: float = xInput * maxSpeed
        y: float = yInput * maxSpeed

        return self.getRawTargetSpeedsWithConstantHeading(
            x, y, angle, currentHeadingAngle
        )

    def getJoystickAngle(self, headingX: float, headingY: float) -> float:
        self.lastAngleScalar = (
            self.lastAngleScalar
            if self.withinHypotDeadband(headingX, headingY)
            else np.atan2(headingX, headingY)
        )
        return self.lastAngleScalar

    def getTargetSpeedsWithHeading(
        self,
        xInput: float,
        yInput: float,
        headingX: float,
        headingY: float,
        currentHeadingAngle: radians,
        maxSpeed: meters_per_second,
    ):
        angle: radians = (
            self.lastAngleScalar
            if self.withinHypotDeadband(headingX, headingY)
            else np.atan2(headingX, headingY)
        )

        speeds: ChassisSpeeds = self.getTargetSpeedsWithAngle(
            xInput, yInput, angle, currentHeadingAngle, maxSpeed
        )

        self.lastAngleScalar = angle

        return speeds

    def getRawTargetSpeedsWithHeadingVelocity(
        self, xSpeed: meters_per_second, ySpeed: meters_per_second, omega: float
    ) -> ChassisSpeeds:
        if self.xLimiter is not None:
            xSpeed = self.xLimiter.calculate(xSpeed)
        if self.yLimiter is not None:
            ySpeed = self.yLimiter.calculate(ySpeed)
        if self.angleLimiter is not None:
            omega = self.angleLimiter.calculate(omega)

        return ChassisSpeeds(xSpeed, ySpeed, omega)

    def getRawTargetSpeedsWithConstantHeading(
        self,
        xSpeed: meters_per_second,
        ySpeed: meters_per_second,
        targetHeadingAngle: radians,
        currentHeadingAngle: radians,
    ) -> ChassisSpeeds:
        return self.getRawTargetSpeedsWithHeadingVelocity(
            xSpeed,
            ySpeed,
            self.headingCalculate(currentHeadingAngle, targetHeadingAngle),
        )

    def headingCalculate(
        self, currentHeadingAngle: radians, targetHeadingAngle: radians
    ) -> float:
        return (
            self.thetaController.calculate(currentHeadingAngle, targetHeadingAngle)
            * self.config.maxAngularVelocity
        )

    def setMaximumChassisAngularVelocity(self, angularVelocity: float) -> None:
        self.config.maxAngularVelocity = angularVelocity


class SwerveDrive:
    # TODO
    def __init__(
        self,
        config: SwerveDriveConfiguration,
        controllerConfig: SwerveControllerConfiguration,
        maxSpeed: meters_per_second,
        startingPose: Pose2d,
    ):
        self.field: Field2d = Field2d()
        self.__attainableMaxTranslationalSpeed: meters_per_second = maxSpeed
        self.__maxChassisSpeed: meters_per_second = maxSpeed
        self.__attainableMaxRotationalVelocity: float = np.pi * 2
        self.swerveDriveConfiguration: Final[SwerveDriveConfiguration] = config
        self.swerveController: SwerveController = SwerveController(controllerConfig)
        self.kinematics: Final[SwerveDrive4Kinematics] = SwerveDrive4Kinematics(
            *config.moduleLocations
        )
        self.__odometryThread: Final[Notifier] = Notifier(self.updateOdometry)
        self.__swerveModules: Final[List[SwerveModule]] = config.modules

        if RobotBase.isSimulation():
            self.__simIMU: SwerveIMUSimulation = SwerveIMUSimulation()
            self.imuReadingCache: Cache[Rotation3d] = Cache(
                self.__simIMU.getGyroRotation3d, 5
            )
        else:
            self.__imu: SwerveIMU = config.imu
            self.__imu.factoryDefault()
            self.imuReadingCache: Cache[Rotation3d] = Cache(self.__imu.getRotation3d, 5)

        self.swerveDrivePoseEstimator: Final[SwerveDrive4PoseEstimator] = (
            SwerveDrive4PoseEstimator(
                self.kinematics, self.getYaw(), self.getModulePositions(), startingPose
            )
        )
        self.zeroGyro()

        if not SwerveDriveTelemetry.verbosity.value >= TelemetryVerbosity.POSE.value:
            SmartDashboard.putData("Field", self.field)
        if SwerveDriveTelemetry.verbosity.value >= TelemetryVerbosity.INFO.value:
            SwerveDriveTelemetry.maxSpeed = maxSpeed
            SwerveDriveTelemetry.maxAngularVelocity = (
                self.swerveController.config.maxAngularVelocity
            )
            SwerveDriveTelemetry.moduleCount = len(self.__swerveModules)
            SwerveDriveTelemetry.sizeFrontBack = metersToInches(
                SwerveMath.getSwerveModuleConfig(
                    self.__swerveModules, True, False
                ).moduleLocation.X()
                + SwerveMath.getSwerveModuleConfig(
                    self.__swerveModules, False, False
                ).moduleLocation.X()
            )
            SwerveDriveTelemetry.sizeLeftRight = metersToInches(
                SwerveMath.getSwerveModuleConfig(
                    self.__swerveModules, False, True
                ).moduleLocation.Y()
                + SwerveMath.getSwerveModuleConfig(
                    self.__swerveModules, False, False
                ).moduleLocation.Y()
            )
            SwerveDriveTelemetry.wheelLocations = (
                [0.0] * SwerveDriveTelemetry.moduleCount * 2
            )
            for module in self.__swerveModules:
                SwerveDriveTelemetry.wheelLocations[module.moduleNumber * 2] = (
                    metersToInches(module.configuration.moduleLocation.X())
                )
                SwerveDriveTelemetry.wheelLocations[(module.moduleNumber * 2) + 1] = (
                    metersToInches(module.configuration.moduleLocation.Y())
                )

            SwerveDriveTelemetry.measuredStates = (
                [0.0] * SwerveDriveTelemetry.moduleCount * 2
            )
            SwerveDriveTelemetry.desiredStates = (
                [0.0] * SwerveDriveTelemetry.moduleCount * 2
            )
            SwerveDriveTelemetry.desiredStatesObj = [
                SwerveModuleState()
            ] * SwerveDriveTelemetry.moduleCount
            SwerveDriveTelemetry.measuredStatesObj = [
                SwerveModuleState()
            ] * SwerveDriveTelemetry.moduleCount

        self.__odometryLock: Final[thrd.RLock] = thrd.RLock()
        self.__tunerXRecommendation: Final[Alert] = Alert(
            "Swerve Drive",
            "Your Swerve Drive is compatible with Tuner X swerve generator, please consider using that instead of YAGSL. More information here!\n"
            + "https://pro.docs.ctr-electronics.com/en/latest/docs/tuner/tuner-swerve/index.html",
            Alert.AlertType.kWarning,
        )
        self.__rawIMUPublisher: Final[DoublePublisher] = (
            NetworkTableInstance.getDefault()
            .getTable("SmartDashboard")
            .getDoubleTopic("swerve/imu/raw")
            .publish()
        )
        self.__adjustedIMUPublisher: Final[DoublePublisher] = (
            NetworkTableInstance.getDefault()
            .getTable("SmartDashboard")
            .getDoubleTopic("swerve/imu/adjusted")
            .publish()
        )
        self.chassisVelocityCorrection: bool = True
        self.autonomousChassisVelocityCorrection: bool = False
        self.angularVelocityCorrection: bool = False
        self.autonomousAngularVelocityCorrection: bool = False
        self.angularVelcoityCoefficient: float = 0
        self.headingCorrection: bool = False
        self.__discretizationdt: seconds = 0.02
        self.__moduleSynchronizationCounter: int = 0
        self.__lastHeading: radians = 0
        self.__HEADING_CORRECTION_DEADBAND: float = 0.01

        self.setOdometryPeriod(0.004 if SwerveDriveTelemetry.isSimulation else 0.02)
        self.checkIfTunerXCompatible()

        hal.report(
            hal.tResourceType.kResourceType_RobotDrive.value,
            hal.tInstances.kRobotDriveSwerve_YAGSL.value,
        )

    def updateCacheValidityPeriod(
        self, imu: milliseconds, driveMotor: milliseconds, absoluteEcnoder: milliseconds
    ) -> None:
        self.imuReadingCache.updateValidityPeriod(imu)
        for module in self.__swerveModules:
            module.drivePositionCache.updateValidityPeriod(driveMotor)
            module.driveVelocityCache.updateValidityPeriod(driveMotor)
            module.absolutePositionCache.updateValidityPeriod(absoluteEcnoder)

    def checkIfTunerXCompatible(self) -> None:
        # TODO: waiting for motor impls
        ...
        # comaptible: bool = self.__imu is Pegion2Swerve
        # for module in self.__swerveModules:
        #     compatible = compatible && (module.getDriveMotor() is TalonFXSwerve && module.getAngleMotor() is TalonFXSwerve && module.getAbsoluteEncoder() is CANCoderSwerve)
        #     if (not compatible)
        #         break
        #
        # if (comaptible):
        #     self.__tunerXRecommendation.set(True)
        #

    def setOdometryPeriod(self, period: seconds) -> None:
        self.__odometryThread.stop()
        self.__odometryThread.startPeriodic(period)

    def stopOdometryThread(self) -> None:
        self.__odometryThread.stop()

    def setAngleMotorConversionFactor(self, conversionFactor: float) -> None:
        for module in self.__swerveModules:
            module.setAngleMotorConversionFactor(conversionFactor)

    def setDriveMotorConversionFactor(self, conversionFactor: float) -> None:
        for module in self.__swerveModules:
            module.setDriveMotorConversionFactor(conversionFactor)

    def getOdometryHeading(self) -> Rotation2d:
        return self.swerveDrivePoseEstimator.getEstimatedPosition().rotation()

    def setHeadingCorrecrtion(self, state: bool, deadband: Optional[float]) -> None:
        if deadband is None:
            deadband = self.__HEADING_CORRECTION_DEADBAND
        self.headingCorrection = state
        self.__HEADING_CORRECTION_DEADBAND = deadband

    def driveFieldOrientedAndRobotOriented(
        self, fieldOrientedVelocity: ChassisSpeeds, robotOrientedVelocity: ChassisSpeeds
    ) -> None:
        self.drive(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                fieldOrientedVelocity, self.getOdometryHeading()
            )
            + robotOrientedVelocity
        )

    def driveFieldOriented(
        self,
        fieldRelativeSpeeds: ChassisSpeeds,
        centerOfRotation: Optional[Translation2d],
    ) -> None:
        if centerOfRotation is not None:
            self.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    fieldRelativeSpeeds, self.getOdometryHeading()
                ),
                centerOfRotation,
            )
        else:
            self.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    fieldRelativeSpeeds, self.getOdometryHeading()
                )
            )

    def drive(
        self, velocity: ChassisSpeeds, centerOfRotation: Translation2d = Translation2d()
    ) -> None:
        self.driveVelocities(velocity, False, centerOfRotation)

    def driveAroundControl(
        self,
        translation: Translation2d,
        rotation: float,
        fieldRelative: bool,
        isOpenLoop: bool,
        centerOfRotation: Translation2d,
    ) -> None:
        velocity: ChassisSpeeds = ChassisSpeeds(
            translation.X(), translation.Y(), rotation
        )

        if fieldRelative:
            velocity = ChassisSpeeds.fromFieldRelativeSpeeds(
                velocity, self.getOdometryHeading()
            )

        self.driveVelocities(velocity, isOpenLoop, centerOfRotation)

    def driveControl(
        self,
        translation: Translation2d,
        rotation: float,
        fieldRelative: bool,
        isOpenLoop: bool,
    ) -> None:
        velocity: ChassisSpeeds = ChassisSpeeds(
            translation.X(), translation.Y(), rotation
        )

        if fieldRelative:
            velocity = ChassisSpeeds.fromFieldRelativeSpeeds(
                velocity, self.getOdometryHeading()
            )

        self.driveVelocities(velocity, isOpenLoop, Translation2d())

    def driveVelocities(
        self,
        robotRelativeVelocity: ChassisSpeeds,
        isOpenLoop: bool,
        centerOfRotation: Translation2d,
    ) -> None:
        SwerveDriveTelemetry.startCtrlCycle()
        robotRelativeVelocity = self.movementOptimization(
            robotRelativeVelocity,
            self.chassisVelocityCorrection,
            self.angularVelocityCorrection,
        )

        if self.headingCorrection:
            if abs(
                robotRelativeVelocity.omega
            ) > self.__HEADING_CORRECTION_DEADBAND and (
                abs(robotRelativeVelocity.vx) > self.__HEADING_CORRECTION_DEADBAND
                or abs(robotRelativeVelocity.vy) > self.__HEADING_CORRECTION_DEADBAND
            ):
                robotRelativeVelocity.omega = self.swerveController.headingCalculate(
                    self.getOdometryHeading().radians(), self.__lastHeading
                )

            else:
                self.__lastHeading = self.getOdometryHeading().radians()

        if SwerveDriveTelemetry.verbosity.value >= TelemetryVerbosity.LOW.value:
            SwerveDriveTelemetry.desiredChassisSpeedsObj = robotRelativeVelocity

        swerveModuleStates: tuple[
            SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState
        ] = self.kinematics.toSwerveModuleStates(
            robotRelativeVelocity, centerOfRotation
        )

        self.setRawModuleStates(swerveModuleStates, robotRelativeVelocity, isOpenLoop)

    def setMaximumAttainableSpeeds(
        self,
        attainableMaxTranslationalSpeed: meters_per_second,
        attainableMaxRotationalVelocity: float,
    ) -> None:
        self.__attainableMaxTranslationalSpeed = attainableMaxTranslationalSpeed
        self.__attainableMaxRotationalVelocity = attainableMaxRotationalVelocity

    def setMaximumAllowableSpeeds(
        self, maxTranslationalSpeed: meters_per_second, maxRotationalVelocity: float
    ) -> None:
        self.__maxChassisSpeed = maxTranslationalSpeed
        self.swerveController.config.maxAngularVelocity = maxRotationalVelocity

    def getMaximumChassisVelocity(self) -> meters_per_second:
        return min(self.__attainableMaxTranslationalSpeed, self.__maxChassisSpeed)

    def getMaximumModuleDriveVelocity(self) -> meters_per_second:
        return self.__swerveModules[0].getMaxDriveVelocity()

    def getMaximumModuleAngleVelocity(self) -> degrees_per_second:
        return self.__swerveModules[0].getMaxAngularVelocity()

    def getMaximumChassisAngularVelocity(self) -> float:
        return min(
            self.__attainableMaxRotationalVelocity,
            self.swerveController.config.maxAngularVelocity,
        )

    def setRawModuleStates(
        self,
        desiredStates: tuple[
            SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState
        ],
        desiredChassisSpeeds: ChassisSpeeds,
        isOpenLoop: bool,
    ) -> None:
        maxModuleSpeed: meters_per_second = self.getMaximumModuleDriveVelocity()
        if (
            self.__attainableMaxTranslationalSpeed != 0
            or self.__attainableMaxRotationalVelocity != 0
        ) and self.__attainableMaxTranslationalSpeed != self.__maxChassisSpeed:
            desiredStates = SwerveDrive4Kinematics.desaturateWheelSpeeds(
                desiredStates,
                desiredChassisSpeeds,
                maxModuleSpeed,
                self.__attainableMaxTranslationalSpeed,
                self.__attainableMaxRotationalVelocity,
            )
        else:
            desiredStates = SwerveDrive4Kinematics.desaturateWheelSpeeds(
                desiredStates, maxModuleSpeed
            )

        for module in self.__swerveModules:
            module.setDesiredState(
                desiredStates[module.moduleNumber], isOpenLoop, False
            )

    def setModuleStates(
        self,
        desiredStates: tuple[
            SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState
        ],
        isOpenLoop: bool,
    ):
        SwerveDriveTelemetry.startCtrlCycle()
        maxModuleSpeed: meters_per_second = self.getMaximumModuleDriveVelocity()
        desiredStates = self.kinematics.toSwerveModuleStates(
            self.kinematics.toChassisSpeeds(desiredStates)
        )
        desiredStates = SwerveDrive4Kinematics.desaturateWheelSpeeds(
            desiredStates, maxModuleSpeed
        )

        for module in self.__swerveModules:
            module.setDesiredState(
                desiredStates[module.moduleNumber], isOpenLoop, False
            )

    def driveFeedforward(
        self,
        robotRelativeVelocity: ChassisSpeeds,
        states: tuple[
            SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState
        ],
        feedforwardForces: list[newtons],
    ) -> None:
        SwerveDriveTelemetry.startCtrlCycle()
        if SwerveDriveTelemetry.verbosity.value >= TelemetryVerbosity.LOW.value:
            SwerveDriveTelemetry.desiredChassisSpeedsObj = robotRelativeVelocity

        for module in self.__swerveModules:
            driveMotorModel: DCMotor = module.configuration.driveMotor.getSimMotor()
            driveGearRatio: float = (
                module.configuration.conversionFactors.drive.gearRatio
            )
            wheelRadius: meters = (
                inchesToMeters(module.configuration.conversionFactors.drive.diameter)
                / 2
            )

            desiredGround: meters_per_second = states[module.moduleNumber].speed
            feedforwardVoltage: volts = driveMotorModel.voltage(
                feedforwardForces[module.moduleNumber] * wheelRadius / driveGearRatio,
                desiredGround / wheelRadius * driveGearRatio,
            )

            module.applyDesiredState(
                states[module.moduleNumber], False, feedforwardVoltage
            )

    def setChassisSpeeds(self, robotRelativeSpeeds: ChassisSpeeds) -> None:
        SwerveDriveTelemetry.startCtrlCycle()
        robotRelativeSpeeds = self.movementOptimization(
            robotRelativeSpeeds,
            self.autonomousChassisVelocityCorrection,
            self.autonomousAngularVelocityCorrection,
        )
        SwerveDriveTelemetry.desiredChassisSpeedsObj = robotRelativeSpeeds

        self.setRawModuleStates(
            self.kinematics.toSwerveModuleStates(robotRelativeSpeeds),
            robotRelativeSpeeds,
            False,
        )

    def getPose(self) -> Pose2d:
        poseEstimation: Pose2d = Pose2d()
        with self.__odometryLock:
            poseEstimation: Pose2d = (
                self.swerveDrivePoseEstimator.getEstimatedPosition()
            )

        return poseEstimation

    def getSimulatedDriveTrainPose(self) -> Pose2d:
        return Pose2d()

    def getFieldVelocity(self) -> ChassisSpeeds:
        robotRelativeSpeeds: ChassisSpeeds = self.kinematics.toChassisSpeeds(
            self.getStates()
        )
        return ChassisSpeeds.fromRobotRelativeSpeeds(
            robotRelativeSpeeds, self.getOdometryHeading()
        )

    def getRobotVelocity(self) -> ChassisSpeeds:
        return self.kinematics.toChassisSpeeds(self.getStates())

    def resetOdometry(self, pose: Pose2d) -> None:
        with self.__odometryLock:
            self.swerveDrivePoseEstimator.resetPosition(
                self.getYaw(), self.getModulePositions(), pose
            )

            if SwerveDriveTelemetry.isSimulation:
                ...

        robotRelativeSpeeds: ChassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            ChassisSpeeds(0, 0, 0), self.getYaw()
        )
        self.kinematics.toSwerveModuleStates(robotRelativeSpeeds)

    def postTrajectory(self, trajectory: Trajectory) -> None:
        if SwerveDriveTelemetry.verbosity.value >= TelemetryVerbosity.POSE.value:
            self.field.getObject("Trajectory").setTrajectory(trajectory)

    def getStates(
        self,
    ) -> tuple[
        SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState
    ]:
        states: list[SwerveModuleState] = [
            SwerveModuleState()
        ] * self.swerveDriveConfiguration.moduleCount

        for module in self.__swerveModules:
            states[module.moduleNumber] = module.getState()

        return states[0], states[1], states[2], states[3]

    def getModulePositions(
        self,
    ) -> Tuple[
        SwerveModulePosition,
        SwerveModulePosition,
        SwerveModulePosition,
        SwerveModulePosition,
    ]:
        positions: List[SwerveModulePosition] = [
            SwerveModulePosition()
        ] * self.swerveDriveConfiguration.moduleCount

        for module in self.__swerveModules:
            positions[module.moduleNumber] = module.getPosition()

        return positions[0], positions[1], positions[2], positions[3]

    def getGyro(self) -> SwerveIMU:
        return self.swerveDriveConfiguration.imu

    def setGyro(self, gyro: Rotation3d) -> None:
        if SwerveDriveTelemetry.isSimulation:
            self.setGyroOffset(self.__simIMU.getGyroRotation3d() - gyro)
        else:
            self.setGyroOffset(self.__imu.getRawRotation3d() - gyro)

        self.imuReadingCache.update()

    def zeroGyro(self) -> None:
        if SwerveDriveTelemetry.isSimulation:
            self.__simIMU.setAngle(0)
        else:
            self.setGyroOffset(self.__imu.getRawRotation3d())

        self.imuReadingCache.update()
        self.swerveController.lastAngleScalar = 0
        self.__lastHeading = 0
        self.resetOdometry(Pose2d(self.getPose().translation(), Rotation2d()))

    def getYaw(self) -> Rotation2d:
        return Rotation2d(self.imuReadingCache.getValue().Z())

    def getPitch(self) -> Rotation2d:
        return Rotation2d(self.imuReadingCache.getValue().Y())

    def getRoll(self) -> Rotation2d:
        return Rotation2d(self.imuReadingCache.getValue().X())

    def getRotation3d(self) -> Rotation3d:
        return self.imuReadingCache.getValue()

    def getAccel(self) -> Optional[Translation3d]:
        if SwerveDriveTelemetry.isSimulation:
            return self.__simIMU.getAccel()
        else:
            return self.__imu.getAccel()

    def setMotorIdleMode(self, brake: bool) -> None:
        for module in self.__swerveModules:
            module.setMotorBrake(brake)

    def setModuleEncoderAutoSynchronize(self, enabled: bool, deadband: degrees) -> None:
        for module in self.__swerveModules:
            module.setEncoderAutoSynchronize(enabled, deadband)

    def lockPose(self) -> None:
        for module in self.__swerveModules:
            desiredState: SwerveModuleState = SwerveModuleState(
                0, module.configuration.moduleLocation.angle()
            )
            if SwerveDriveTelemetry.verbosity.value >= TelemetryVerbosity.INFO.value:
                SwerveDriveTelemetry.desiredStatesObj[module.moduleNumber] = (
                    desiredState
                )

            module.setDesiredState(desiredState, False, True)

        self.kinematics.toSwerveModuleStates(ChassisSpeeds())

    def getSwerveModulePoses(self, robotPose: Pose2d) -> List[Pose2d]:
        poses: List[Pose2d] = []

        for module in self.__swerveModules:
            poses.append(
                robotPose
                + Transform2d(
                    module.configuration.moduleLocation, module.getState().angle
                )
            )

        return poses

    def replaceSwerveModuleFeedforward(
        self, driveFeedforward: SimpleMotorFeedforwardMeters
    ) -> None:
        for module in self.__swerveModules:
            module.setFeedForward(driveFeedforward)

    def updateOdometry(self) -> None:
        SwerveDriveTelemetry.startOdomCycle()
        self.__odometryLock.acquire()
        try:
            self.swerveDrivePoseEstimator.update(
                self.getYaw(), self.getModulePositions()
            )

            if SwerveDriveTelemetry.isSimulation:
                ...
            if SwerveDriveTelemetry.verbosity.value >= TelemetryVerbosity.INFO.value:
                SwerveDriveTelemetry.measuredChassisSpeedsObj = self.getRobotVelocity()
                SwerveDriveTelemetry.robotRotationObj = self.getOdometryHeading()

            if SwerveDriveTelemetry.verbosity.value >= TelemetryVerbosity.POSE.value:
                if SwerveDriveTelemetry.isSimulation:
                    ...
                else:
                    self.field.setRobotPose(
                        self.swerveDrivePoseEstimator.getEstimatedPosition()
                    )

            sumVelocity: float = 0
            for module in self.__swerveModules:
                moduleState: SwerveModuleState = module.getState()
                sumVelocity += abs(moduleState.speed)

                if (
                    SwerveDriveTelemetry.verbosity.value
                    == TelemetryVerbosity.HIGH.value
                ):
                    module.updateTelemetry()
                    self.__rawIMUPublisher.set(self.getYaw().degrees())
                    self.__adjustedIMUPublisher.set(self.getOdometryHeading().degrees())

                if (
                    SwerveDriveTelemetry.verbosity.value
                    >= TelemetryVerbosity.INFO.value
                ):
                    SwerveDriveTelemetry.measuredStatesObj[module.moduleNumber] = (
                        moduleState
                    )

            self.__moduleSynchronizationCounter += 1
            if sumVelocity <= 0.01 and self.__moduleSynchronizationCounter > 5:
                self.synchronizeModuleEncoders()
                self.__moduleSynchronizationCounter = 0

            if SwerveDriveTelemetry.verbosity.value >= TelemetryVerbosity.INFO.value:
                SwerveDriveTelemetry.updateData()

        except Exception as e:
            self.__odometryLock.release()
            raise e

        self.__odometryLock.release()
        SwerveDriveTelemetry.endOdomCycle()

    def invalidateCache(self) -> None:
        self.imuReadingCache.update()
        for module in self.__swerveModules:
            module.invalidateCache()

    def synchronizeModuleEncoders(self) -> None:
        for module in self.__swerveModules:
            module.queueSynchronizeEncoders()

    def setGyroOffset(self, offset: Rotation3d) -> None:
        if SwerveDriveTelemetry.isSimulation:
            self.__simIMU.setAngle(offset.Z())
        else:
            self.__imu.setOffset(offset)

        self.imuReadingCache.update()

    def addVisionMeasurement(
        self,
        robotPose: Pose2d,
        timestamp: seconds,
        visionMeasurementStdDevs: Optional[tuple[float, float, float]] = None,
    ) -> None:
        with self.__odometryLock:
            if visionMeasurementStdDevs is not None:
                self.swerveDrivePoseEstimator.addVisionMeasurement(
                    robotPose, timestamp, visionMeasurementStdDevs
                )
            else:
                self.swerveDrivePoseEstimator.addVisionMeasurement(robotPose, timestamp)

    def setVisionMeasurementStdDevs(
        self, visionMeasurementStdDevs: tuple[float, float, float]
    ) -> None:
        with self.__odometryLock:
            self.swerveDrivePoseEstimator.setVisionMeasurementStdDevs(
                visionMeasurementStdDevs
            )

    def getSwerveController(self) -> SwerveController:
        return self.swerveController

    def getModules(self) -> List[SwerveModule]:
        return self.swerveDriveConfiguration.modules

    def getModuleMap(self) -> Dict[str, SwerveModule]:
        moduleMap: Dict[str, SwerveModule] = {}
        for module in self.__swerveModules:
            moduleMap[module.configuration.name] = module
        return moduleMap

    def resetDriveEncoders(self) -> None:
        for module in self.__swerveModules:
            module.getDriveMotor().setPosition(0)

    def pushOffsetsToEncoders(self) -> None:
        for module in self.__swerveModules:
            module.pushOffsetsToEncoders()

    def restoreInternalOffset(self) -> None:
        for module in self.__swerveModules:
            module.restoreInternalOffset()

    def setAutoCenteringModules(self, enabled: bool) -> None:
        for module in self.__swerveModules:
            module.setAntiJitter(not enabled)

    def setCosineCompensator(self, enabled: bool) -> None:
        for module in self.__swerveModules:
            module.configuration.useCosineCompensator = enabled

    def setChassisDiscretization(self, enable: bool, dt: seconds) -> None:
        if not SwerveDriveTelemetry.isSimulation:
            self.chassisVelocityCorrection = enable
            self.__discretizationdt = dt

    def setChassisDiscretizationForMode(
        self, useInTeleop: bool, useInAuto: bool, dt: seconds
    ) -> None:
        if not SwerveDriveTelemetry.isSimulation:
            self.chassisVelocityCorrection = useInTeleop
            self.autonomousChassisVelocityCorrection = useInAuto
            self.__discretizationdt = dt

    def setAngularVelocityCompensation(
        self, useInTeleop: bool, useInAuto: bool, angularVelocityCoeff: float
    ) -> None:
        if not SwerveDriveTelemetry.isSimulation:
            self.angularVelcoityCoefficient = angularVelocityCoeff
            self.autonomousAngularVelocityCorrection = useInAuto
            self.angularVelocityCorrection = useInTeleop

    def angularVelocitySkewCorrection(
        self, robotRelativeVelocity: ChassisSpeeds
    ) -> ChassisSpeeds:
        angularVelocity: Rotation2d = Rotation2d(
            self.__imu.getYawAngularVelocity() * self.angularVelcoityCoefficient
        )
        if angularVelocity.radians() != 0.0:
            fieldRelativeVelocity: ChassisSpeeds = (
                ChassisSpeeds.fromRobotRelativeSpeeds(
                    robotRelativeVelocity, self.getOdometryHeading()
                )
            )
            robotRelativeVelocity = ChassisSpeeds.fromFieldRelativeSpeeds(
                fieldRelativeVelocity, self.getOdometryHeading() + angularVelocity
            )

        return robotRelativeVelocity

    def movementOptimization(
        self,
        robotRelativeVelocity: ChassisSpeeds,
        useChassisDiscretize: bool,
        useAngularVelocitySkewCorrection: bool,
    ) -> ChassisSpeeds:
        if useAngularVelocitySkewCorrection:
            robotRelativeVelocity = self.angularVelocitySkewCorrection(
                robotRelativeVelocity
            )

        if useChassisDiscretize:
            robotRelativeVelocity = ChassisSpeeds.discretize(
                robotRelativeVelocity, self.__discretizationdt
            )

        return robotRelativeVelocity

    def toSwerveModuleStates(
        self, robotRelativeVelocity: ChassisSpeeds, optimize: bool
    ) -> tuple[
        SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState
    ]:
        if optimize:
            robotRelativeVelocity = self.movementOptimization(
                robotRelativeVelocity,
                self.chassisVelocityCorrection,
                self.angularVelocityCorrection,
            )

        return self.kinematics.toSwerveModuleStates(robotRelativeVelocity)


class SwerveInputMode(Enum):
    TRANSLATION_ONLY = 0
    ANGULAR_VELOCITY = 1
    HEADING = 2
    AIM = 3


class SwerveInputStream:
    def __init__(
        self,
        drive: SwerveDrive,
        x: FloatSupplier,
        y: FloatSupplier,
        rot: Optional[FloatSupplier] = None,
        headingX: Optional[FloatSupplier] = None,
        headingY: Optional[FloatSupplier] = None,
    ) -> None:
        self.__controllerTranslationX: Final[FloatSupplier] = x
        self.__controllerTranslationY: Final[FloatSupplier] = y
        self.__swerveDrive: Final[SwerveDrive] = drive
        self.__controllerOmega: Optional[FloatSupplier] = rot
        self.__controllerHeadingX: Optional[FloatSupplier] = headingX
        self.__controllerHeadingY: Optional[FloatSupplier] = headingY

        self.__axisDeadband: Optional[float] = None
        self.__translationAxisScale: Optional[float] = None
        self.__omegaAxisScale: Optional[float] = None
        self.__aimTarget: Optional[Pose2d] = None
        self.__headingEnabled: Optional[BooleanSupplier] = None
        self.__lockedHeading: Optional[Rotation2d] = None
        self.__aimEnabled: Optional[BooleanSupplier] = None
        self.__translationOnlyEnabled: Optional[BooleanSupplier] = None
        self.__translationCube: Optional[BooleanSupplier] = None
        self.__omegaCube: Optional[BooleanSupplier] = None
        self.__robotRelative: Optional[BooleanSupplier] = None
        self.__allianceRelative: Optional[BooleanSupplier] = None
        self.__headingOffsetEnabled: Optional[BooleanSupplier] = None
        self.__headingOffset: Optional[Rotation2d] = None
        self.__swerveController: SwerveController
        self.__currentMode: SwerveInputMode = SwerveInputMode.ANGULAR_VELOCITY

    @classmethod
    def of(
        cls, drive: SwerveDrive, x: FloatSupplier, y: FloatSupplier
    ) -> "SwerveInputStream":
        return SwerveInputStream(drive=drive, x=x, y=y)

    def copy(self) -> "SwerveInputStream":
        newStream: SwerveInputStream = SwerveInputStream(
            self.__swerveDrive,
            x=self.__controllerTranslationX,
            y=self.__controllerTranslationY,
        )
        newStream.__controllerOmega = self.__controllerOmega
        newStream.__controllerHeadingX = self.__controllerHeadingX
        newStream.__controllerHeadingY = self.__controllerHeadingY
        newStream.__axisDeadband = self.__axisDeadband
        newStream.__translationAxisScale = self.__translationAxisScale
        newStream.__omegaAxisScale = self.__omegaAxisScale
        newStream.__aimTarget = self.__aimTarget
        newStream.__headingEnabled = self.__headingEnabled
        newStream.__aimEnabled = self.__aimEnabled
        newStream.__currentMode = self.__currentMode
        newStream.__translationOnlyEnabled = self.__translationOnlyEnabled
        newStream.__lockedHeading = self.__lockedHeading
        newStream.__swerveController = self.__swerveController
        newStream.__omegaCube = self.__omegaCube
        newStream.__translationCube = self.__translationCube
        newStream.__robotRelative = self.__robotRelative
        newStream.__allianceRelative = self.__allianceRelative
        newStream.__headingOffsetEnabled = self.__headingOffsetEnabled
        newStream.__headingOffset = self.__headingOffset
        return newStream

    def robotRelativeEnabled(
        self, enabled: Union[BooleanSupplier, bool]
    ) -> "SwerveInputStream":
        if isinstance(enabled, Callable):
            self.__robotRelative = enabled
        elif isinstance(enabled, bool):
            self.__robotRelative = (lambda: enabled) if enabled else None
        return self

    def headingOffsetEnabled(
        self, enabled: Union[BooleanSupplier, bool]
    ) -> "SwerveInputStream":
        if isinstance(enabled, Callable):
            self.__headingOffsetEnabled = enabled
        elif isinstance(enabled, bool):
            self.__headingOffsetEnabled = (lambda: enabled) if enabled else None
        return self

    def witHeadingOffset(self, angle: Rotation2d) -> "SwerveInputStream":
        self.__headingOffset = angle
        return self

    # TODO finish chaining methods
