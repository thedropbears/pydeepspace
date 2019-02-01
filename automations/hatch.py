from magicbot import StateMachine, state, timed_state

from automations.alignment import Aligner
from components.hatch import Hatch


class HatchController(StateMachine):
    hatch: Hatch
    align: Aligner

    def punch(self, force=False):
        self.engage(force=force)

    @state(first=True, must_finish=True)
    def aligning(self, initial_call):
        self.align.engage()
        if self.align.successful and not self.align.is_executing:
            self.next_state("punching")

    @state(must_finish=True)
    def punching(self):
        self.hatch.punch()
        self.next_state("retracting")

    @timed_state(must_finish=True, duration=1.2)
    def retracting(self, state_tm):
        if state_tm > 1:
            self.hatch.retract()
            self.done()
