import wpilib
import magicbot


class Arm:

    main_piston: wpilib.Solenoid
    helper_piston: wpilib.Solenoid

    def __init__(self):
        self.main_piston_on = True
        self.helper_piston_on = False
        self.last_main_piston_on = None
        self.last_helper_piston_on = None

    def execute(self):
        if self.main_piston_on != self.last_main_piston_on:
            self.main_piston.set(self.main_piston_on)
        if self.helper_piston_on != self.last_helper_piston_on:
            self.helper_piston.set(self.helper_piston_on)

    def raise_main_piston(self):
        self.main_piston_on = True

    def lower_main_piston(self):
        self.main_piston_on = False

    def raise_helper_piston(self):
        self.helper_piston_on = True

    def lower_helper_piston(self):
        self.helper_piston_on = False

    def move_arm_down_state(self):
        if self.main_piston_on and not self.helper_piston_on:
            self.raise_helper_piston()
            self.lower_main_piston()
        elif not self.main_piston_on and self.helper_piston_on:
            self.lower_helper_piston()
            self.lower_main_piston()
    
    def move_arm_up_state(self):
        if not self.main_piston_on and self.helper_piston_on:
            self.raise_main_piston()
            self.lower_helper_piston()
        elif not self.main_piston_on and not self.helper_piston_on:
            self.raise_main_piston()
            self.raise_helper_piston()
            self.lower_main_piston()