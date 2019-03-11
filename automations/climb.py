from magicbot import StateMachine, state, timed_state
from components.climb import Climber
from components.cargo import CargoManipulator, Height
from pyswervedrive.chassis import SwerveChassis


class ClimbAutomation(StateMachine):

    chassis: SwerveChassis
    climber: Climber
    cargo_component: CargoManipulator
    VERBOSE_LOGGING = True

    def on_disable(self):
        self.done()

    def start_climb_lv3(self):
        self.engage()

    def done(self):
        super().done()
        self.chassis.set_modules_drive_brake()
        self.chassis.automation_running = False

    @state(first=True, must_finish=True)
    def extend_both_lifts_lv3(self, initial_call):
        if initial_call:
            self.chassis.set_modules_drive_coast()
            self.chassis.heading_hold_off()
            self.chassis.automation_running = True

            self.cargo_component.move_to(Height.LOADING_STATION)

        self.move_swerves()
        self.climber.extend_all()

        if self.climber.is_both_extended():
            self.next_state_now("align_front_lift")

    @timed_state(must_finish=True, next_state="retract_front_lift", duration=1)
    def align_front_lift(self):
        self.climber.drive_forward()

        if self.climber.is_front_touching_podium():
            self.next_state("retract_front_lift")

    @state(must_finish=True)
    def retract_front_lift(self):
        self.climber.retract_front()

        if self.climber.front.is_above_ground():
            self.next_state_now("align_back_lift")

    @timed_state(must_finish=True, next_state="roll_back", duration=2)
    def align_back_lift(self):
        self.move_swerves(0.5)
        self.climber.drive_forward()

    @state
    def roll_back(self, initial_call):
        if initial_call:
            self.start_odometry = self.chassis.odometry_y
        self.move_swerves(-0.3)
        self.climber.drive_forwards(-0.2)
        if abs(self.chassis.odometry_y-self.start_odometry) > 0.05:
            self.next_state("retract_back_lift")
            self.move_swerves(0)

    @state(must_finish=True)
    def retract_back_lift(self, initial_call):
        if initial_call:
            self.climber.fire_pistons()

        self.climber.drive_forward()
        self.move_swerves(0)
        self.climber.retract_back()

        if self.climber.back.is_retracted():
            self.done()

    def move_swerves(self, velocity=0.05):
        self.chassis.set_inputs(0, velocity, 0, field_oriented=False)
