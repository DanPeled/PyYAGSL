from abc import ABC, abstractmethod
from typing import Any, Optional, Union, Self
from wpimath.system.plant import DCMotor
from wpimath.units import (
    volts,
    meters_per_second,
    degrees_per_second,
    degrees,
    meters,
    amperes,
    seconds,
)

from swervelib.encoders import SwerveAbsoluteEncoder
from swervelib.parser.pidf import PIDFConfig


class SwerveMotor(ABC):
    def __init__(self):
        self.maximumRetries: int = 5
        self.simMotor: DCMotor
        self.__isDriveMotor: bool

    @abstractmethod
    def factoryDefaults(self) -> None: ...

    @abstractmethod
    def clearStickyFaults(self) -> None: ...

    @abstractmethod
    def setAbsoluteEncoder(self, encoder: Optional[SwerveAbsoluteEncoder]) -> Self: ...

    @abstractmethod
    def configureIntegratedEncoder(self, positionConversionFactor: float) -> None: ...

    @abstractmethod
    def configurePIDF(self, config: PIDFConfig) -> None: ...

    @abstractmethod
    def configurePIDWrapping(self, minInput: float, maxOutput: float) -> None: ...

    @abstractmethod
    def setMotorBrake(self, isBrakeMode: bool) -> None: ...

    @abstractmethod
    def setInverted(self, inverted: bool) -> None: ...

    @abstractmethod
    def burnFlash(self) -> None: ...

    @abstractmethod
    def set(self, precentOutput: float) -> None: ...

    @abstractmethod
    def setReference(
        self, setpoint: float, feedforward: float, position=-1
    ) -> None: ...

    @abstractmethod
    def getVoltage(self) -> volts: ...

    @abstractmethod
    def setVoltage(self, voltage: volts) -> None: ...

    @abstractmethod
    def getAppliedOutput(self) -> float: ...

    @abstractmethod
    def getVelocity(self) -> Union[meters_per_second, degrees_per_second]: ...

    @abstractmethod
    def getPosition(self) -> Union[meters, degrees]: ...

    @abstractmethod
    def setPosition(self, position: Union[meters, degrees]) -> None: ...

    @abstractmethod
    def setVoltageCompensation(self, minimalVoltage: volts) -> None: ...

    @abstractmethod
    def setCurrentLimit(self, currentLimit: amperes) -> None: ...

    @abstractmethod
    def setLoopRampRate(self, rampRate: seconds) -> None: ...

    @abstractmethod
    def getMotor(self) -> Any: ...

    @abstractmethod
    def getSimMotor(self) -> DCMotor: ...

    @abstractmethod
    def isAttachedAbsoluteEncoder(self) -> bool: ...
