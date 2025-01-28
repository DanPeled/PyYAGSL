from typing import Optional

from wpilib import Field2d
from wpimath.geometry import Pose2d, Rotation2d, Rotation3d, Translation3d
from wpimath.kinematics import (
    SwerveDrive4Kinematics,
    SwerveModulePosition,
    SwerveModuleState,
)
from wpimath.units import radians, volts

from swervelib.parser.moduleConfig import SwerveModulePhysicalCharacteristics


class SwerveIMUSimulation:
    # TODO implement actual sim IMU
    def getYaw(self) -> Rotation2d:
        return Rotation2d()

    def getPitch(self) -> Rotation2d:
        return Rotation2d()

    def getRoll(self) -> Rotation2d:
        return Rotation2d()

    def getGyroRotation3d(self) -> Rotation3d:
        return Rotation3d(0, 0, self.getYaw().radians())

    def getAccel(self) -> Optional[Translation3d]:
        return None

    def updateOdometry(
        self,
        kinematics: SwerveDrive4Kinematics,
        states: list[SwerveModuleState],
        modulePoses: list[Pose2d],
        field: Field2d,
    ) -> None:
        field.getObject("XModules").setPoses(modulePoses)

    def setAngle(self, angle: radians) -> None: ...


class SwerveModuleSimulation:
    # TODO Implement actual Module Sim
    def configureSimModule(
        self,
        physicalCharacteristics: SwerveModulePhysicalCharacteristics,
    ) -> None: ...

    def updateStateAndPosition(self, desiredState: SwerveModuleState) -> None: ...

    def runDriveMotorCharacterization(
        self, desiredFacing: Rotation2d, suppliedVolts: volts
    ) -> None: ...

    def runAngleMotorCharacterization(self, suppliedVolts: volts) -> None: ...

    def getPosition(self) -> SwerveModulePosition: ...

    def getState(self) -> SwerveModuleState: ...
