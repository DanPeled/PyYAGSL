from typing import Final

from wpimath.geometry import Translation2d
from wpimath.system.plant import DCMotor
from wpimath.units import meters, meters_per_second

from swervelib.imu import SwerveIMU
from swervelib.math import SwerveMath
from swervelib.parser.moduleConfig import (
    SwerveModuleConfiguration,
    SwerveModulePhysicalCharacteristics,
)
from swervelib.parser.pidf import PIDFConfig
from swervelib.swerve import SwerveModule


class SwerveDriveConfiguration:
    def __init__(
        self,
        moduleConfigs: list[SwerveModuleConfiguration],
        swerveIMU: SwerveIMU,
        invertedIMU: bool,
        physicalCharacteristics: SwerveModulePhysicalCharacteristics,
    ):
        self.moduleCount: Final[int] = len(moduleConfigs)
        self.imu: SwerveIMU = swerveIMU
        self.imu.setInverted(invertedIMU)
        self.modules: list[SwerveModule]
        self.moduleLocations: list[Translation2d] = [Translation2d()] * len(
            moduleConfigs
        )
        for module in self.modules:
            self.moduleLocations[module.moduleNumber] = (
                module.configuration.moduleLocation
            )
        self.physicalCharacteristics: SwerveModulePhysicalCharacteristics = (
            physicalCharacteristics
        )

    def createModules(
        self, swerves: list[SwerveModuleConfiguration]
    ) -> list[SwerveModule]:
        modList: list[SwerveModule] = []

        for i in range(len(swerves)):
            modList.insert(i, SwerveModule(i, swerves[i]))

        return modList

    def getDriveBaseRadius(self) -> meters:
        centerOfModules: Translation2d = self.moduleLocations[0]

        for i in range(1, len(self.moduleLocations)):
            centerOfModules += self.moduleLocations[i]

        return centerOfModules.distance(self.moduleLocations[0])

    def getTrackWidth(self) -> meters:
        fr: SwerveModuleConfiguration = SwerveMath.getSwerveModuleConfig(
            self.modules, True, False
        )
        fl: SwerveModuleConfiguration = SwerveMath.getSwerveModuleConfig(
            self.modules, True, True
        )

        return fr.moduleLocation.distance(fl.moduleLocation)

    def getTrackLength(self) -> meters:
        br: SwerveModuleConfiguration = SwerveMath.getSwerveModuleConfig(
            self.modules, False, False
        )
        bl: SwerveModuleConfiguration = SwerveMath.getSwerveModuleConfig(
            self.modules, False, True
        )

        return br.moduleLocation.distance(bl.moduleLocation)

    def getDriveMotorSim(self) -> DCMotor:
        fl = SwerveMath.getSwerveModuleConfig(self.modules, True, True)
        return fl.driveMotor.getSimMotor()

    def getAngleMotorSim(self) -> DCMotor:
        fl = SwerveMath.getSwerveModuleConfig(self.modules, True, True)
        return fl.angleMotor.getSimMotor()


class SwerveControllerConfiguration:
    def __init__(
        self,
        driveCfg: SwerveDriveConfiguration,
        headingPIDF: PIDFConfig,
        maxSpeed: meters_per_second,
        angleJoystickRadiusDeadband: float = 0.5,
    ):
        self.maxAngularVelocity: float = SwerveMath.calculateMaxAngularVelocity(
            maxSpeed,
            abs(driveCfg.moduleLocations[0].X()),
            abs(driveCfg.moduleLocations[0].Y()),
        )

        self.headingPIDF: Final[PIDFConfig] = headingPIDF
        self.angleJoystickRadiusDeadband: Final[float] = angleJoystickRadiusDeadband
