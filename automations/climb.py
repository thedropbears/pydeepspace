from magicbot import StateMachine, state
from components.climb import Lift, LiftDrive


class ClimbAutomation(StateMachine):

    front_lift: Lift
    back_lift: Lift

    lift_drive: LiftDrive

    def start_climb(self):
        self.engage()

    @state(first=True, must_finish=True)
    def extend_both_lifts(self, initial_call):
        if initial_call:
            self.front_lift.extend()
            self.back_lift.extend()
        if self.front_lift.is_at_set_pos() and self.back_lift.is_at_set_pos():
            self.next_state_now("align_front_lift")

    def align_front_lift(self, initial_call):
        if initial_call:
            self.lift_drive.move()
        if self.front_lift.is_touching_podium():
            self.lift_drive.stop()
            self.next_state_now("retract_front_lift")

    def retract_front_lift(self, initial_call):
        if initial_call:
            self.front_lift.retract()
        if self.front_lift.is_at_set_pos():
            self.next_state_now("align_back_lift")

    def align_back_lift(self, initial_call):
        if initial_call:
            self.lift_drive.move()
        if self.back_lift.is_touching_podium():
            self.lift_drive.stop()
            self.next_state_now("retract_back_lift")

    def retract_back_lift(self, initial_call):
        if initial_call:
            self.back_lift.retract()
        if self.back_lift.is_at_set_pos():
            self.done()
