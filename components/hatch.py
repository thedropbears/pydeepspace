import wpilib
import ctre


class Hatch:
    top_puncher: wpilib.Solenoid
    left_puncher: wpilib.Solenoid
    right_puncher: wpilib.Solenoid
    actuator_arm: ctre.TalonSRX

    def __init__(self):
        self.puncher = False
        self.retracter = True

    def setup(self):
        """This is called after variables are injected by magicbot."""

    def on_enable(self):
        """This is called whenever the robot transitions to being enabled."""

    def execute(self):
        """Run at the end of every control loop iteration."""

    def punch(self):
        self.top_puncher.set(True)
        self.left_puncher.set(True)
        self.right_puncher.set(True)

    def retract(self):
        self.top_puncher.set(False)
        self.left_puncher.set(False)
        self.right_puncher.set(False)