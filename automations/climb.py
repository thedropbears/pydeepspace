from magicbot import StateMachine, state
from components.climb import Climber
from components.cargo import Arm, Height
from pyswervedrive.chassis import SwerveChassis


class ClimbAutomation(StateMachine):

    chassis: SwerveChassis
    climber: Climber
    arm: Arm

    def start_climb_lv3(self):
        self.engage(initial_state="extend_both_lifts_lv3")

    def done(self):
        super().done()
        self.chassis.set_modules_drive_brake()
        self.chassis.automation_running = False
        self.climber.stop_all()

    @state(first=True, must_finish=True)
    def extend_both_lifts_lv3(self, initial_call, state_tm):
        self.move_swerves()

        if initial_call:
            self.climber.extend_all()

            self.chassis.set_modules_drive_coast()
            self.chassis.heading_hold_off()
            self.chassis.automation_running = True

            self.arm.move_to(Height.LOADING_STATION)

        if self.climber.is_both_extended():
            self.climber.stop_all()
            self.next_state_now("align_front_lift")

    @state(must_finish=True)
    def align_front_lift(self, initial_call):
        self.move_swerves()

        if initial_call:
            self.climber.move_wheels()
        if self.climber.is_front_touching_podium():
            self.climber.stop_wheels()
            self.next_state_now("retract_front_lift")

    @state(must_finish=True)
    def retract_front_lift(self, initial_call):
        self.move_swerves()

        if initial_call:
            self.climber.retract_front()
        if self.climber.is_front_above_ground_level():
            self.next_state_now("align_back_lift")

    @state(must_finish=True)
    def align_back_lift(self, initial_call):
        self.move_swerves()

        if initial_call:
            self.climber.move_wheels()
        if self.climber.is_back_touching_podium():
            self.next_state_now("fire_pistons")

    @state(must_finish=True)
    def fire_pistons(self):
        self.move_swerves()

        self.climber.fire_solenoid()
        self.next_state_now("retract_back_lift")

    @state(must_finish=True)
    def retract_back_lift(self, initial_call):
        self.move_swerves()

        if initial_call:
            self.climber.retract_back()
        if self.climber.is_back_retracted():
            self.climber.stop_back()
            self.climber.stop_wheels()
            self.done()

    def move_swerves(self):
        self.chassis.set_inputs(0, 0.05, 0, field_oriented=False)
