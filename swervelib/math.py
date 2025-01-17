from wpimath.geometry import Rotation2d, Translation3d
from wpimath.units import (
    kilograms,
    meters,
    degrees,
    volts,
    meters_per_second,
    meters_per_second_squared,
    newton_meters,
)
from wpimath.controller import SimpleMotorFeedforwardMeters
import numpy as np


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
        ka: float = optimalVoltage / SwerveMath.calculateMaxAcceleration(
            wheelGripCoefficientOfFriction
        )

        return SimpleMotorFeedforwardMeters(0, kv, 0)

    @staticmethod
    def calculateMaxAngularVelocity(
        maxSpeed: meters_per_second, furthestModuleX: meters, furthestModuleY: meters
    ):
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
    ): ...  # TODO (waiting for config class)
