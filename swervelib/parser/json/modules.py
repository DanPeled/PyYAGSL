from swervelib.math import SwerveMath
from wpimath.units import inches, inchesToMeters


class AngleConversionFactorsJson:
    def __init__(self):
        self.gearRatio: float
        self.factor: float = 0

    def calculate(self) -> float:
        if self.factor == 0:
            self.factor = SwerveMath.calculateDegreesPerSteeringRotation(self.gearRatio)
        return self.factor


class DriveConversionFactorsJson:
    def __init__(self):
        self.gearRatio: float
        self.diameter: inches
        self.factor: float = 0

    def calculate(self) -> float:
        if self.factor == 0:
            self.factor = SwerveMath.calculateMetersPerRotation(
                inchesToMeters(self.diameter), self.gearRatio
            )

        return self.factor


class ConversionFactorsJson:
    def __init__(self):
        self.drive: DriveConversionFactorsJson = DriveConversionFactorsJson()
        self.angle: AngleConversionFactorsJson = AngleConversionFactorsJson()

    def isDriveEmpty(self) -> bool:
        self.drive.calculate()
        return self.drive.factor == 0

    def isAngleEmpty(self) -> bool:
        self.angle.calculate()
        return self.angle.factor == 0

    def works(self) -> bool:
        return (self.angle.factor != 0 and self.drive.factor != 0) or (
            self.drive.gearRatio != 0
            and self.drive.diameter != 0
            and (self.angle.gearRatio != 0)
        )


class BoolMotorJson:
    def __init__(self):
        self.driveInverted: bool
        self.angleInverted: bool


class LocationJson:
    def __init__(self) -> None:
        self.front: float = 0
        self.x: float = 0
        self.left: float = 0
        self.y: float = 0
