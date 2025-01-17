from wpimath.geometry import Translation3d
from wpimath.units import kilograms


class Matter:
    def __init__(self, position: Translation3d, mass: kilograms):
        self.position: Translation3d = position
        self.mass: kilograms = mass

    def massMoment(self) -> Translation3d:
        return self.position * self.mass
