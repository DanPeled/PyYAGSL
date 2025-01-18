from sys import is_stack_trampoline_active, setdlopenflags
from typing import Final, Optional
from wpimath.geometry import Rotation2d
from wpimath.units import (
    inchesToMeters,
    meters_per_second,
    degrees_per_second,
    degrees,
    radiansToRotations,
    rotationsToDegrees,
    volts,
)
import numpy as np
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState
from wpilib import Alert
from wpimath.controller import SimpleMotorFeedforwardMeters
from ntcore import BooleanPublisher, DoublePublisher, NetworkTableInstance
from swervelib.encoders import SwerveAbsoluteEncoder
from swervelib.math import SwerveMath
from swervelib.motors import SwerveMotor
from swervelib.parser.cache import Cache
from swervelib.parser.moduleConfig import SwerveModuleConfiguration
from swervelib.parser.pidf import PIDFConfig, PIDFRange
from swervelib.simDevices import SwerveModuleSimulation
from swervelib.telemetry import SwerveDriveTelemetry, TelemetryVerbosity


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
