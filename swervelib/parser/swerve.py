from typing import Final
from wpimath.units import meters_per_second
from swervelib.parser.pidf import PIDFConfig


class SwerveDriveConfiguration: ...  # TODO


class SwerveControllerConfiguration:
    def __init__(
        self,
        driveCfg: SwerveDriveConfiguration,
        headingPIDf: PIDFConfig,
        angleJoystickRadiusDeadband: float,
        maxSpeed: meters_per_second,
    ) -> None:
        self.headingPIDF: Final[PIDFConfig] = headingPIDf
        self.angleJoystickRadiusDeadband: Final[float] = angleJoystickRadiusDeadband
        self.maxAngularVelocity: float
