import math

from magicbot import StateMachine, state

from components.hatch import Hatch
from pyswervedrive.chassis import SwerveChassis


class HatchAutomation(StateMachine):

    chassis: SwerveChassis
    hatch: Hatch

    def __init__(self):
        super().__init__()
        self.fired_position = 0, 0

    def grab(self):
        self.hatch.extend_fingers()
        self.hatch.has_hatch = True

    def outake(self, force=False):
        self.engage("outaking", force=force)

    @state(first=True, must_finish=True)
    def outaking(self, state_tm, initial_call):
        if initial_call:
            self.hatch.retract_fingers()
        if state_tm > 0.5:
            self.hatch.punch()
            self.next_state("retract_after_move")

    @state(must_finish=True)
    def retract_after_move(self, initial_call, state_tm):
        """
        Ensure we have moved away before we retract punchers.
        """
        if initial_call:
            self.fired_position = self.chassis.position
        if (
            math.hypot(
                self.fired_position[0] - self.chassis.position[0],
                self.fired_position[1] - self.chassis.position[1],
            )
            > 0.5
            or state_tm > 5
        ):
            self.hatch.retract()
            self.done()
