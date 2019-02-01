import wpilib


class Hatch:
    top_puncher: wpilib.Solenoid
    left_puncher: wpilib.Solenoid
    right_puncher: wpilib.Solenoid

    top_limit_switch: wpilib.DigitalInput
    left_limit_switch: wpilib.DigitalInput
    right_limit_switch: wpilib.DigitalInput

    def __init__(self):
        self.punch_on = False

    def execute(self):
        """Run at the end of every control loop iteration."""
        self.top_puncher.set(self.punch_on)
        self.left_puncher.set(self.punch_on)
        self.right_puncher.set(self.punch_on)

    def punch(self):
        self.punch_on = True

    def retract(self):
        self.punch_on = False

    def is_contained(self):
        return any(
            [
                not self.top_limit_switch.get(),
                not self.left_limit_switch.get(),
                not self.right_limit_switch.get(),
            ]
        )
