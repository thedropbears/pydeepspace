import wpilib
import magicbot


class Arm:

    bottom_piston: wpilib.Solenoid
    top_piston: wpilib.Solenoid

    def __init__(self):
        self.bottom_piston_on = True
        self.top_piston_on = True
        self.last_bottom_piston_on = None
        self.last_top_piston_on = None

    def execute(self):
        if self.bottom_piston_on != self.last_bottom_piston_on:
            self.bottom_piston.set(self.bottom_piston_on)
        if self.top_piston_on != self.last_top_piston_on:
            self.top_piston.set(self.top_piston_on)

    def raise_bottom_piston(self):
        self.bottom_piston_on = True

    def lower_bottom_piston(self):
        self.bottom_piston_on = False

    def raise_top_piston(self):
        self.top_piston_on = True

    def lower_top_piston(self):
        self.top_piston_on = False
