from wpimath.controller import PIDController


class PIDFConfig:
    def __init__(
        self, p: float = 0, i: float = 0, d: float = 0, f: float = 0, iz: float = 0
    ):
        self.p: float = p
        self.i: float = i
        self.d: float = d
        self.f: float = f
        self.iz: float = iz
        self.output: PIDFRange = PIDFRange()

    def createPIDController(self) -> PIDController:
        return PIDController(self.p, self.i, self.d)


class PIDFRange:
    def __init__(self):
        self.min: float = -1
        self.max: float = 1
