from typing import Callable, Self
from wpilib import RobotBase, RobotController
from wpimath.units import microseconds, milliseconds


class Cache[T]:
    """
    Cache for frequently requested data.
    """

    def __init__(self, supplier: Callable[[], T], validityPeriod: milliseconds):
        """
        Cache for arbitrary values.

        :param supplier: Callable that provides the cached value.
        :param validityPeriod: Validity period in milliseconds.
        """
        self.value: T = supplier()
        self.supplier: Callable[[], T] = supplier
        self.timestamp: microseconds = RobotController.getFPGATime()
        self.validityPeriod: microseconds = validityPeriod * 1000

    def isStale(self) -> bool:
        """
        Return whether the cache is stale.

        :return: The stale state of the cache.
        """
        return (RobotController.getFPGATime() - self.timestamp) > self.validityPeriod

    def update(self) -> Self:
        """
        Update the cache value and timestamp.

        :return: The Cache instance for chaining.
        """
        self.value = self.supplier()
        self.timestamp = RobotController.getFPGATime()
        return self

    def updateSupplier(self, supplier: Callable[[], T]) -> Self:
        """
        Update the supplier to a new source and refresh the cached value.

        :param supplier: The new supplier source.
        :return: The Cache instance for chaining.
        """
        self.supplier = supplier
        self.update()
        return self

    def updateValidityPeriod(self, validityPeriod: milliseconds) -> Self:
        """
        Update the validity period for the cached value, also updates the value.

        :param validityPeriod: The new validity period in milliseconds.
        :return: The Cache instance for chaining.
        """
        self.validityPeriod = validityPeriod * 1000
        self.update()
        return self

    def getValue(self) -> T:
        """
        Get the most up-to-date cached value.

        :return: The latest cached version of the value.
        """
        if self.isStale() or RobotBase.isSimulation():
            self.update()

        return self.value
