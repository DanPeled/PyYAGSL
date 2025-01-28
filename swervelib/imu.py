from abc import ABC, abstractmethod
from typing import Any, Optional

from wpimath.geometry import Rotation3d, Translation3d
from wpimath.units import degrees_per_second


class SwerveIMU(ABC):
    @abstractmethod
    def factoryDefault(self) -> None: ...

    @abstractmethod
    def clearStickyFaults(self) -> None: ...

    @abstractmethod
    def setOffset(self, offset: Rotation3d) -> None: ...

    @abstractmethod
    def setInverted(self, invertIMU: bool) -> None: ...

    @abstractmethod
    def getRawRotation3d(self) -> Rotation3d: ...

    @abstractmethod
    def getRotation3d(self) -> Rotation3d: ...

    @abstractmethod
    def getAccel(self) -> Optional[Translation3d]: ...

    @abstractmethod
    def getYawAngularVelocity(self) -> degrees_per_second: ...

    @abstractmethod
    def getIMU(self) -> Any: ...
