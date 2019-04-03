import wpilib


class Hatch:

    fingers: wpilib.DoubleSolenoid
    punchers: wpilib.Solenoid
    enable_piston: wpilib.DoubleSolenoid

    left_limit_switch: wpilib.DigitalInput
    right_limit_switch: wpilib.DigitalInput

    def setup(self):
        self.has_hatch = False
        self._fingers_state = wpilib.DoubleSolenoid.Value.kReverse
        self.enable_hatch = False

    def on_enable(self):
        self._punch_on = False
        self.enable_piston.set(wpilib.DoubleSolenoid.Value.kForward)
        self.loop_counter = 0
        self.enable_counter = 0

    def execute(self):
        """Run at the end of every control loop iteration."""
        delay = -1
        self.fingers.set(self._fingers_state)
        self.punchers.set(self._punch_on and self.loop_counter > delay)
        if self._punch_on and self.loop_counter > delay:
            self.has_hatch = False
        self.loop_counter += 1
        self.enable_counter += 1
        if self.enable_hatch:
            if self.enable_counter > 5:
                self.extend_fingers()
                self.has_hatch = True
                self.enable_hatch = False
        # if self.is_contained():
        #     self.has_hatch = True

    def punch(self):
        self.loop_counter = 0
        self._punch_on = True

    def retract(self):
        self._punch_on = False

    def extend_fingers(self):
        self._fingers_state = wpilib.DoubleSolenoid.Value.kForward

    def retract_fingers(self):
        self._fingers_state = wpilib.DoubleSolenoid.Value.kReverse

    def is_contained(self):
        return any(
            [not self.left_limit_switch.get(), not self.right_limit_switch.get()]
        )
