from typing import Final, Optional

import numpy as np
from wpimath.controller import SimpleMotorFeedforwardMeters
from wpimath.geometry import Pose2d, Rotation2d, Translation2d, Translation3d, Twist2d
from wpimath.kinematics import ChassisSpeeds, SwerveModuleState
from wpimath.units import (
    degrees,
    kilograms,
    meters,
    meters_per_second,
    meters_per_second_squared,
    newton_meters,
    radians,
    seconds,
    volts,
)
from swervelib.parser import SwerveDriveConfiguration, SwerveModuleConfiguration
from swervelib.swerve import SwerveModule


class Matter:
    def __init__(self, position: Translation3d, mass: kilograms):
        self.position: Translation3d = position
        self.mass: kilograms = mass

    def massMoment(self) -> Translation3d:
        return self.position * self.mass


class SwerveMath:
    @staticmethod
    def calculateMetersPerRotation(
        wheelDiameter: meters, driveGearRatio: float, pulsePerRotation: float = 1
    ) -> meters:
        return (np.pi * wheelDiameter) / (driveGearRatio * pulsePerRotation)

    @staticmethod
    def normalizeAngle(angle: degrees) -> degrees:
        angleRotation = Rotation2d.fromDegrees(angle)
        return Rotation2d(angleRotation.cos(), angleRotation.sin()).degrees()

    @staticmethod
    def applyDeadband(value: float, scaled: bool, deadband: float) -> float:
        value = value if np.abs(value) > deadband else 0
        return (
            1 / (1 - deadband) * (np.abs(value) - deadband) * np.sign(value)
            if scaled
            else value
        )

    @staticmethod
    def calculateDegreesPerSteeringRotation(
        angleGearRatio: float, pulsePerRotation: float = 1
    ) -> degrees:
        return 360 / (angleGearRatio * pulsePerRotation)

    @staticmethod
    def createDriveFeedForward(
        optimalVoltage: volts,
        maxSpeed: meters_per_second,
        wheelGripCoefficientOfFriction: float,
    ) -> SimpleMotorFeedforwardMeters:
        kv: float = optimalVoltage / maxSpeed
        # ka: float = optimalVoltage / SwerveMath.calculateMaxAcceleration(
        #     wheelGripCoefficientOfFriction
        # )

        return SimpleMotorFeedforwardMeters(0, kv, 0)

    @staticmethod
    def calculateMaxAngularVelocity(
        maxSpeed: meters_per_second, furthestModuleX: meters, furthestModuleY: meters
    ) -> float:
        return maxSpeed / (np.hypot(furthestModuleX, furthestModuleY))

    @staticmethod
    def calculateMaxAcceleration(cof: float) -> float:
        return cof * 9.81

    @staticmethod
    def calculateMaxRobotAcceleration(
        stallTorqueNm: newton_meters,
        gearRatio: float,
        moduleCount: int,
        wheelDiameter: meters,
        robotMass: kilograms,
    ) -> meters_per_second_squared:
        return (stallTorqueNm * gearRatio * moduleCount) / (
            (wheelDiameter / 2) * robotMass
        )

    @staticmethod
    def calcMaxTippingAccel(
        angle: Rotation2d, matter: list[Matter], robotMass: kilograms, config
    ):
        centerMass: Translation3d = Translation3d()
        for obj in matter:
            centerMass += obj.massMoment()
        robotCG: Translation3d = centerMass / robotMass
        horizontalCG: Translation2d = robotCG.toTranslation2d()

        projectedHorizontalCg: Translation2d = Translation2d(
            (angle.sin() * angle.cos() * horizontalCG.Y())
            + ((angle.cos() ** 2) * horizontalCG.X()),
            (angle.sin() * angle.cos() * horizontalCG.X())
            + ((angle.sin() ** 2) * horizontalCG.Y()),
        )

        # Projects the edge of the wheelbase onto the direction line.  Assumes the wheelbase is
        # rectangular.
        # Because a line is being projected, rather than a point, one of the coordinates of the
        # projected point is
        # already known.
        projectedWheelbaseEdge: Optional[Translation2d] = None
        angDeg: degrees = angle.degrees()
        if 45 >= angDeg >= -45:
            conf: SwerveModuleConfiguration = SwerveMath.getSwerveModuleConfig(
                config.modules, True, True
            )
            projectedWheelbaseEdge = Translation2d(
                conf.moduleLocation.X(), conf.moduleLocation.X() * angle.tan()
            )
        elif 135 >= angDeg > 45:
            conf: SwerveModuleConfiguration = SwerveMath.getSwerveModuleConfig(
                config.modules, True, True
            )
            projectedWheelbaseEdge = Translation2d(
                conf.moduleLocation.Y() / angle.tan(), conf.moduleLocation.Y()
            )
        elif -135 <= angDeg < -45:
            conf: SwerveModuleConfiguration = SwerveMath.getSwerveModuleConfig(
                config.modules, True, False
            )
            projectedWheelbaseEdge = Translation2d(
                conf.moduleLocation.Y() / angle.tan(), conf.moduleLocation.Y()
            )
        else:
            conf: SwerveModuleConfiguration = SwerveMath.getSwerveModuleConfig(
                config.modules, False, True
            )
            projectedWheelbaseEdge = Translation2d(
                conf.moduleLocation.X(), conf.moduleLocation.X() * angle.tan()
            )
        horizontalDistance: float = (
            projectedHorizontalCg + projectedWheelbaseEdge
        ).norm()

        return 9.81 * horizontalDistance / robotCG.Z()

    @staticmethod
    def poseLog(transform: Pose2d) -> Twist2d:
        kEps: Final[float] = 1e-9
        dtheta: Final[radians] = transform.rotation().radians()
        half_dtheta: Final[radians] = dtheta * 0.5
        cos_minus_one: Final[float] = transform.rotation().cos() - 1.0
        halftheta_by_tan_of_halfdtheta: float

        if np.abs(cos_minus_one) < kEps:
            halftheta_by_tan_of_halfdtheta = 1.0 - (1.0 / 12.0 * dtheta * dtheta)
        else:
            halftheta_by_tan_of_halfdtheta = (
                -(half_dtheta * transform.rotation().sin()) / cos_minus_one
            )
        translation_part: Final[Translation2d] = transform.translation().rotateBy(
            Rotation2d(halftheta_by_tan_of_halfdtheta, -half_dtheta)
        )

        return Twist2d(translation_part.X(), translation_part.Y(), dtheta)

    @staticmethod
    def limitVelocity(
        commandedVelocity: Translation2d,
        fieldVelocity: ChassisSpeeds,
        robotPose: Pose2d,
        loopTime: seconds,
        robotMass: kilograms,
        matter: list[Matter],
        config: SwerveDriveConfiguration,
    ) -> Translation2d: ...  # TODO waiting for SwerveController class

    @staticmethod
    def getSwerveModuleConfig(
        modules: list[SwerveModule], front: bool, left: bool
    ) -> SwerveModuleConfiguration:
        target: Translation2d = modules[0].configuration.moduleLocation
        current: Translation2d
        temp: Translation2d
        configuration: SwerveModuleConfiguration = modules[0].configuration

        for module in modules:
            current = module.configuration.moduleLocation
            if front:
                temp = current if target.Y() >= current.Y() else target
            else:
                temp = current if target.Y() <= current.Y() else target

            if left:
                target = temp if target.X() >= temp.X() else target
            else:
                target = temp if target.X() <= temp.X() else target

            configuration = module.configuration if current == target else configuration

        return configuration

    @staticmethod
    def placeInAppropriate0To360Scope(
        scopeReference: degrees, newAngle: degrees
    ) -> degrees:
        diffRevs: degrees = np.round((scopeReference - newAngle) / 360) * 360
        return diffRevs + newAngle

    @staticmethod
    def antiJitter(
        moduleState: SwerveModuleState,
        lastModuleState: SwerveModuleState,
        maxSpeed: float,
    ) -> None:
        if np.abs(moduleState.speed) <= (maxSpeed * 0.01):
            moduleState.angle = lastModuleState.angle

    @staticmethod
    def cubeTranslation(translation: Translation2d) -> Translation2d:
        if np.hypot(translation.X(), translation.Y()) <= 1.0e-6:
            return translation
        return Translation2d(translation.norm() ** 3, translation.angle())

    @staticmethod
    def scaleTranslation(translation: Translation2d, scalar: float) -> Translation2d:
        if np.hypot(translation.X(), translation.Y()) <= 1.0e-6:
            return translation
        return Translation2d(translation.norm() * scalar, translation.angle())
