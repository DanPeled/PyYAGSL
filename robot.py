from wpilib import TimedRobot
import wpimath.units


class Robot(TimedRobot):
    def __init__(self, period: wpimath.units.seconds = 0.02) -> None:
        super().__init__(period)
