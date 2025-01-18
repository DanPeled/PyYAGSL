from swervelib.parser.pidf import PIDFConfig


class ControllerPropertiesJson:
    def __init__(self):
        self.angleJoystickRadiusDeadband: float
        self.headingPIDF: PIDFConfig
