from abc import ABC, abstractmethod
from typing import Any
from wpimath.units import degrees, degrees_per_second


class SwerveAbsoluteEncoder(ABC):
    def __init__(self):
        self.maximumRetries: int = 5
        self.readingError: bool = False

    @abstractmethod
    def factoryDefault(self) -> None: ...

    @abstractmethod
    def clearStickyFaults(self) -> None: ...

    @abstractmethod
    def configure(self, inverted: bool) -> None: ...

    @abstractmethod
    def getAbsolutePosition(self) -> degrees: ...

    @abstractmethod
    def getAbsoluteEncoder(self) -> Any: ...

    @abstractmethod
    def setAbsoluteEncoderOffset(self, offset: float) -> bool: ...

    @abstractmethod
    def getVelocity(self) -> degrees_per_second: ...
