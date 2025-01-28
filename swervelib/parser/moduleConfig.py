from typing import Final, Optional

from wpimath.geometry import Translation2d
from wpimath.units import (
    amperes,
    degrees,
    inches,
    inchesToMeters,
    kilogram_square_meters,
    kilograms,
    meters,
    seconds,
    volts,
)

from swervelib.encoders import SwerveAbsoluteEncoder
from swervelib.math import SwerveMath
from swervelib.motors import SwerveMotor
from swervelib.parser.pidf import PIDFConfig


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


class SwerveModulePhysicalCharacteristics:
    def __init__(
        self,
        conversionFactors: Optional[ConversionFactorsJson],
        driveMotorRampRate: seconds,
        angleMotorRampRate: seconds,
        wheelGripCoefficientOfFriction: float = 1.19,
        optimalVoltage: volts = 12,
        driveMotorCurrentLimit: amperes = 40,
        angleMotorCurrentLimit: amperes = 20,
        driveFrictionVoltage: volts = 0.2,
        angleFrictionVoltage: volts = 0.3,
        steerRotationalInertia: kilogram_square_meters = 0.03,
        robotMass: kilograms = 50,
    ):
        self.driveMotorCurrentLimit: Final[amperes] = driveMotorCurrentLimit
        self.angleMotorCurrentLimit: Final[amperes] = angleMotorCurrentLimit
        self.driveMotorRampRate: Final[seconds] = driveMotorRampRate
        self.angleMotorRampRate: Final[seconds] = angleMotorRampRate
        self.driveFrictionVoltage: Final[volts] = driveFrictionVoltage
        self.angleFrictionVoltage: Final[volts] = angleFrictionVoltage
        self.wheelGripCoefficientOfFriction: Final[float] = (
            wheelGripCoefficientOfFriction
        )
        self.steerRotationalInertia: Final[kilogram_square_meters] = (
            steerRotationalInertia
        )
        self.robotMass: Final[kilograms] = robotMass
        self.optimalVoltage: volts = optimalVoltage

        self.conversionFactors: Optional[ConversionFactorsJson] = conversionFactors

        if conversionFactors is not None:
            if conversionFactors.isAngleEmpty() and conversionFactors.isDriveEmpty():
                self.conversionFactors = None


class SwerveModuleConfiguration:
    def __init__(
        self,
        driveMotor: SwerveMotor,
        angleMotor: SwerveMotor,
        conversionFactors: ConversionFactorsJson,
        absoluteEncoder: SwerveAbsoluteEncoder,
        angleOffset: degrees,
        x: meters,
        y: meters,
        anglePIDF: PIDFConfig,
        velocityPIDF: PIDFConfig,
        physicalCharacteristics: SwerveModulePhysicalCharacteristics,
        name: str,
        useCosineCompensator: bool,
        absoluteEncoderInverted: bool = False,
        driveMotorInverted: bool = False,
        angleMotorInverted: bool = False,
    ):
        self.conversionFactors: Final[ConversionFactorsJson] = conversionFactors
        self.angleOffset: Final[degrees] = angleOffset
        self.absoluteEncoderInverted: Final[bool] = absoluteEncoderInverted
        self.driveMotorInverted: Final[bool] = driveMotorInverted
        self.angleMotorInverted: Final[bool] = angleMotorInverted
        self.anglePIDF: PIDFConfig = anglePIDF
        self.velocityPIDF: PIDFConfig = velocityPIDF
        self.moduleLocation: Translation2d = Translation2d(x, y)
        self.physicalCharacteristics: SwerveModulePhysicalCharacteristics = (
            physicalCharacteristics
        )
        self.driveMotor: SwerveMotor = driveMotor
        self.angleMotor: SwerveMotor = angleMotor
        self.absoluteEncoder: SwerveAbsoluteEncoder = absoluteEncoder
        self.name: str = name
        self.useCosineCompensator: bool = useCosineCompensator
