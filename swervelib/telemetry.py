from enum import Enum
from typing import Final
from ntcore._ntcore import StringPublisher
from wpilib import Alert, DriverStation
from ntcore import (
    DoublePublisher,
    DoubleArrayPublisher,
    NetworkTableInstance,
    StructPublisher,
)
from wpilib import RobotBase
from wpimath.geometry import Rotation2d
from wpimath.kinematics import ChassisSpeeds, SwerveModuleState
from wpilib import Timer


class TelemetryVerbosity(Enum):
    NONE = 0
    LOW = 1
    INFO = 2
    POSE = 3
    HIGH = 4
    MACHINE = 5


class SwerveDriveTelemetry:
    canIdWarning: Final[Alert] = Alert(
        "JSON",
        "CAN IDs greater than 40 can cause undefined behaviour, please use a CAN ID below 40!",
        Alert.AlertType.kWarning,
    )

    i2cLockupWarning: Final[Alert] = Alert(
        "IMU",
        "I2C lockup issue detected on roboRIO. Check console for more information.",
        Alert.AlertType.kWarning,
    )

    serialCommsIssueWarning: Final[Alert] = Alert(
        "IMU",
        "Serial comms is interrupted with USB and other serial traffic and causes intermittent connected/disconnection issues. Please consider another protocol or be mindful of this.",
        Alert.AlertType.kWarning,
    )

    __moduleCountPublisher: Final[DoublePublisher] = (
        NetworkTableInstance.getDefault()
        .getTable("SmartDashboard")
        .getDoubleTopic("swerve/moduleCount")
        .publish()
    )

    __measuredStatesArrayPublisher: Final[DoubleArrayPublisher] = (
        NetworkTableInstance.getDefault()
        .getTable("SmartDashboard")
        .getDoubleArrayTopic("swerve/measuredStates")
        .publish()
    )

    __desiredStatesArrayPublisher: Final[DoubleArrayPublisher] = (
        NetworkTableInstance.getDefault()
        .getTable("SmartDashboard")
        .getDoubleArrayTopic("swerve/desiredStates")
        .publish()
    )

    __measuredChassisSpeedsArrayPublisher: Final[DoubleArrayPublisher] = (
        NetworkTableInstance.getDefault()
        .getTable("SmartDashboard")
        .getDoubleArrayTopic("swerve/measuredChassisSpeeds")
        .publish()
    )

    __desiredChassisSpeedsArrayPublisher: Final[DoubleArrayPublisher] = (
        NetworkTableInstance.getDefault()
        .getTable("SmartDashboard")
        .getDoubleArrayTopic("swerve/desiredChassisSpeeds")
        .publish()
    )

    __robotRotationPublisher: Final[DoublePublisher] = (
        NetworkTableInstance.getDefault()
        .getTable("SmartDashboard")
        .getDoubleTopic("swerve/robotRotation")
        .publish()
    )

    __maxAngularVelocityPublisher: Final[DoublePublisher] = (
        NetworkTableInstance.getDefault()
        .getTable("SmartDashboard")
        .getDoubleTopic("swerve/maxAngularVelocity")
        .publish()
    )

    __measuredStatesStruct: Final[StructPublisher] = (
        NetworkTableInstance.getDefault()
        .getTable("SmartDashboard")
        .getStructTopic("swerve/advantagescope/currentStates", SwerveModuleState)
        .publish()
    )

    __desiredStatesStruct: Final[StructPublisher] = (
        NetworkTableInstance.getDefault()
        .getTable("SmartDashboard")
        .getStructTopic("swerve/advantagescope/desiredStates", SwerveModuleState)
        .publish()
    )

    __measuredChassisSpeedsStruct: Final[StructPublisher] = (
        NetworkTableInstance.getDefault()
        .getTable("SmartDashboard")
        .getStructTopic("swerve/advantagescope/measuredChassisSpeeds", ChassisSpeeds)
        .publish()
    )

    __desiredChassisSpeedsStruct: Final[StructPublisher] = (
        NetworkTableInstance.getDefault()
        .getTable("SmartDashboard")
        .getStructTopic("swerve/advantagescope/desiredChassisSpeeds", ChassisSpeeds)
        .publish()
    )

    __robotRotationStruct: Final[StructPublisher] = (
        NetworkTableInstance.getDefault()
        .getTable("SmartDashboard")
        .getStructTopic("swerve/advantagescope/robotRotation", Rotation2d)
        .publish()
    )

    __wheelLocationsArrayPublisher: Final[DoubleArrayPublisher] = (
        NetworkTableInstance.getDefault()
        .getTable("SmartDashboard")
        .getDoubleArrayTopic("swerve/wheelLocation")
        .publish()
    )

    __maxSpeedPublisher: Final[DoublePublisher] = (
        NetworkTableInstance.getDefault()
        .getTable("SmartDashboard")
        .getDoubleTopic("swerve/maxSpeed")
        .publish()
    )

    __rotationUnitPublisher: Final[StringPublisher] = (
        NetworkTableInstance.getDefault()
        .getTable("SmartDashboard")
        .getStringTopic("swerve/rotationUnit")
        .publish()
    )

    __sizeLeftRightPublisher: Final[DoublePublisher] = (
        NetworkTableInstance.getDefault()
        .getTable("SmartDashboard")
        .getDoubleTopic("swerve/sizeLeftRight")
        .publish()
    )

    __sizeFrontBackPublisher: Final[DoublePublisher] = (
        NetworkTableInstance.getDefault()
        .getTable("SmartDashboard")
        .getDoubleTopic("swerve/sizeFrontBack")
        .publish()
    )

    __forwardDirectionPublisher: Final[StringPublisher] = (
        NetworkTableInstance.getDefault()
        .getTable("SmartDashboard")
        .getStringTopic("swerve/forwardDirection")
        .publish()
    )

    __odomCycleTime: Final[DoublePublisher] = (
        NetworkTableInstance.getDefault()
        .getTable("SmartDashboard")
        .getDoubleTopic("swerve/odomCycleMS")
        .publish()
    )

    __ctrlCycleTime: Final[DoublePublisher] = (
        NetworkTableInstance.getDefault()
        .getTable("SmartDashboard")
        .getDoubleTopic("swerve/controlCycleMS")
        .publish()
    )

    __odomTimer: Final[Timer] = Timer()

    __ctrlTimer: Final[Timer] = Timer()

    measuredStatesObj: list[SwerveModuleState] = [SwerveModuleState()] * 4

    desiredStatesObj: list[SwerveModuleState] = [SwerveModuleState()] * 4

    measuredChassisSpeedsObj: ChassisSpeeds = ChassisSpeeds()

    desiredChassisSpeedsObj: ChassisSpeeds = ChassisSpeeds()

    robotRotationObj: Rotation2d = Rotation2d()

    verbosity: TelemetryVerbosity = TelemetryVerbosity.MACHINE

    isSimulation: bool = RobotBase.isSimulation()

    moduleCount: int

    wheelLocations: list[float]

    measuredStates: list[float]

    desiredStates: list[float]

    robotRotation: float = 0

    maxSpeed: float

    rotationUnit: str = "degrees"

    sizeLeftRight: float

    sizeFrontBack: float

    forwardDirection: str = "up"

    maxAngularVelocity: float

    measuredChassisSpeeds: list[float] = [0.0] * 3

    desiredChassisSpeeds: list[float] = [0.0] * 3

    updateSettings: bool = True

    @classmethod
    def startCtrlCycle(cls) -> None:
        if cls.__ctrlTimer.isRunning():
            cls.__ctrlTimer.reset()
        else:
            cls.__ctrlTimer.start()

    @classmethod
    def endCtrlCycle(cls) -> None:
        if (
            DriverStation.isTeleopEnabled()
            or DriverStation.isAutonomousEnabled()
            or DriverStation.isTestEnabled()
        ):
            cls.__ctrlCycleTime.set(cls.__ctrlTimer.get() * 1000)

        cls.__ctrlTimer.reset()

    @classmethod
    def startOdomCycle(cls) -> None:
        if cls.__odomTimer.isRunning():
            cls.__odomTimer.reset()
        else:
            cls.__odomTimer.start()

    @classmethod
    def endOdomCycle(cls) -> None:
        if (
            DriverStation.isTeleopEnabled()
            or DriverStation.isAutonomousEnabled()
            or DriverStation.isTestEnabled()
        ):
            cls.__odomCycleTime.set(cls.__odomTimer.get() * 1000)

        cls.__odomTimer.reset()

    @classmethod
    def updateSwerveTelemetrySettings(cls) -> None:
        if cls.updateSettings:
            cls.updateSettings = False
            cls.__wheelLocationsArrayPublisher.set(cls.wheelLocations)
            cls.__maxSpeedPublisher.set(cls.maxSpeed)
            cls.__rotationUnitPublisher.set(cls.rotationUnit)
            cls.__sizeLeftRightPublisher.set(cls.sizeLeftRight)
            cls.__sizeFrontBackPublisher.set(cls.sizeFrontBack)
            cls.__forwardDirectionPublisher.set(cls.forwardDirection)

    @classmethod
    def updateData(cls) -> None:
        if cls.updateSettings:
            cls.updateSwerveTelemetrySettings()

        cls.measuredChassisSpeeds[0] = cls.measuredChassisSpeedsObj.vx
        cls.measuredChassisSpeeds[1] = cls.measuredChassisSpeedsObj.vy
        cls.measuredChassisSpeeds[2] = cls.measuredChassisSpeedsObj.omega_dps

        cls.desiredChassisSpeeds[0] = cls.desiredChassisSpeedsObj.vx
        cls.desiredChassisSpeeds[1] = cls.desiredChassisSpeedsObj.vy
        cls.desiredChassisSpeeds[2] = cls.desiredChassisSpeedsObj.omega_dps

        cls.robotRotation = cls.robotRotationObj.degrees()

        for i in range(len(cls.measuredStatesObj)):
            state: SwerveModuleState = cls.measuredStatesObj[i]
            if state is not None:
                cls.measuredStates[i * 2] = state.angle.degrees()
                cls.measuredStates[i * 2 + 1] = state.speed

        for i in range(len(cls.desiredStatesObj)):
            state: SwerveModuleState = cls.desiredStatesObj[i]
            if state is not None:
                cls.desiredStates[i * 2] = state.angle.degrees()
                cls.desiredStates[i * 2 + 1] = state.speed

        cls.__moduleCountPublisher.set(cls.moduleCount)
        cls.__measuredStatesArrayPublisher.set(cls.measuredStates)
        cls.__desiredStatesArrayPublisher.set(cls.desiredStates)
        cls.__robotRotationPublisher.set(cls.robotRotation)
        cls.__maxAngularVelocityPublisher.set(cls.maxAngularVelocity)

        cls.__measuredChassisSpeedsArrayPublisher.set(cls.measuredChassisSpeeds)
        cls.__desiredChassisSpeedsArrayPublisher.set(cls.desiredChassisSpeeds)

        cls.__desiredStatesStruct.set(cls.desiredStatesObj)
        cls.__measuredStatesStruct.set(cls.measuredStatesObj)
        cls.__desiredChassisSpeedsStruct.set(cls.desiredChassisSpeedsObj)
        cls.__measuredChassisSpeedsStruct.set(cls.measuredChassisSpeedsObj)
        cls.__robotRotationStruct.set(cls.robotRotationObj)
