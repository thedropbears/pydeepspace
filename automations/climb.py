from magicbot import StateMachine, state
from components.climb import Climber


class ClimbAutomation(StateMachine):

    climber: Climber

    def start_climb(self):
        self.engage()

    @state(first=True, must_finish=True)
    def extend_both_lifts(self, initial_call):
        if initial_call:
            self.climber.extend_all()
        if self.climber.is_front_at_set_pos() and self.climber.is_back_at_set_pos():
            self.climber.stop_all()
            self.next_state_now("align_front_lift")

    @state(must_finish=True)
    def align_front_lift(self, initial_call):
        if initial_call:
            self.climber.move_wheels()
        if self.climber.is_front_touching_podium():
            self.climber.stop_wheels()
            self.next_state_now("retract_front_lift")

    @state(must_finish=True)
    def retract_front_lift(self, initial_call):
        if initial_call:
            self.climber.retract_front()
        if self.climber.is_front_at_set_pos():
            self.climber.stop_front()
            self.next_state_now("align_back_lift")

    @state(must_finish=True)
    def align_back_lift(self, initial_call):
        if initial_call:
            self.climber.move_wheels()
        if self.climber.is_back_touching_podium():
            self.climber.stop_wheels()
            self.next_state_now("retract_back_lift")

    @state(must_finish=True)
    def retract_back_lift(self, initial_call):
        if initial_call:
            self.climber.retract_back()
        if self.climber.is_back_at_set_pos():
            self.climber.stop_back()
            self.done()
