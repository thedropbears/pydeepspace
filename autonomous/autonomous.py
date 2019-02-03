import math

from magicbot.state_machine import AutonomousStateMachine, state

from automations.alignment import HatchDepositAligner, HatchIntakeAligner
from components.hatch import Hatch
from components.vision import Vision
from pyswervedrive.chassis import SwerveChassis
from utilities.navx import NavX
from utilities.pure_pursuit import PurePursuit


def reflect_2d_y(v: tuple) -> tuple:
    return (v[0], -v[1], v[2], v[3])


class AutoBase(AutonomousStateMachine):

    # Here magicbot injects components
    hatch_deposit: HatchDepositAligner
    hatch_intake: HatchIntakeAligner

    chassis: SwerveChassis
    hatch: Hatch
    imu: NavX
    vision: Vision

    # This one is just a typehint
    pursuit: PurePursuit

    def __init__(self):
        super().__init__()
        self.front_cargo_bay = (5.6 - SwerveChassis.LENGTH / 2 - 0.5, 0.2, 0, 0.2)
        self.setup_loading_bay = (3.3, 3.3, math.pi, 2)
        self.loading_bay = (0.2 + SwerveChassis.LENGTH / 2 + 0.5, 3.38, math.pi, 0.2)
        self.loading_bay_speedup_point = (1 + SwerveChassis.LENGTH / 2, 3.38, math.pi, 2)
        self.side_cargo_bay = (6.6, 0.9 + SwerveChassis.WIDTH / 2 + 0.5, -math.pi/2, 0.2)
        self.side_cargo_bay_slowdown_point = (6.6, 1.6 + SwerveChassis.WIDTH / 2, -math.pi/2, 1.5)
        # The waypoints we use to move to the other side of the field
        self.cross_point = (4.7, 1, 0, 2)
        # points on opposite side of field
        # self.opp_front_cargo_bay = reflect_2d_y(self.front_cargo_bay)
        # self.opp_setup_loading_bay = reflect_2d_y(self.setup_loading_bay)
        self.opp_loading_bay = reflect_2d_y(self.loading_bay)
        self.opp_side_cargo_bay = reflect_2d_y(self.side_cargo_bay)
        self.opp_cross_point = (4.7, -1.8, 0, 2)

        self.start_pos = (
            1.2 + SwerveChassis.LENGTH / 2,
            0 + SwerveChassis.WIDTH / 2,
            0,
            2,
        )
        self.start_speedup = (
            2.5 + SwerveChassis.LENGTH / 2,
            0 + SwerveChassis.WIDTH / 2,
            0,
            2,
        )
        self.completed_runs = 0
        self.desired_angle = 0
        self.desired_angle_navx = 0
        self.minimum_path_completion = 0.85

        self.pursuit = PurePursuit(look_ahead=0.2)

    def on_enable(self):
        super().on_enable()
        self.chassis.odometry_x = self.start_pos[0]
        self.chassis.odometry_y = self.start_pos[1]
        # print(f"odometry = {self.current_pos}")

    @state(first=True)
    def drive_to_cargo_bay(self, initial_call):
        if initial_call:
            # print(f"odometry = {self.current_pos}")
            if self.completed_runs == 0:
                self.pursuit.build_path(
                    (self.current_pos, self.start_speedup, self.front_cargo_bay)
                )
            elif self.completed_runs == 1:
                self.pursuit.build_path(
                    (
                        self.current_pos,
                        self.loading_bay,
                        self.loading_bay_speedup_point,
                        self.side_cargo_bay_slowdown_point,
                        self.side_cargo_bay,
                    )
                )
            elif self.completed_runs == 2:
                self.chassis.set_heading_sp(math.pi)
                self.pursuit.build_path((self.current_pos, self.opp_loading_bay, self.opp_side_cargo_bay))
        if self.pursuit.completed_path and self.completed_runs > 3:
            self.next_state("stop")
        self.follow_path()
        if (
            not math.isnan(self.vision.target_tape_error) and self.ready_for_vision()
        ) or self.pursuit.completed_path:
            self.next_state("deposit_hatch")
            self.completed_runs += 1

    @state
    def deposit_hatch(self, initial_call):
        self.hatch_deposit.engage()
        if not self.hatch.has_hatch:
            self.next_state("drive_to_loading_bay")

    @state
    def drive_to_loading_bay(self, initial_call):
        if initial_call:
            if self.completed_runs == 1:
                self.pursuit.build_path(
                    (self.current_pos, (self.current_pos[0]-0.5, self.current_pos[1], self.imu.getAngle(), 1.5), self.setup_loading_bay, self.loading_bay)
                )
            elif self.completed_runs == 2:
                # we only have a quater field, stop here
                self.next_state("stop")
                return
                self.pursuit.build_path(
                    (
                        self.current_pos,
                        self.cross_point,
                        self.opp_cross_point,
                        self.opp_loading_bay,
                    )
                )
            elif self.completed_runs == 3:
                # return to the loading bay for start of teleop
                self.pursuit.build_path((self.current_pos, self.opp_loading_bay))
        if self.pursuit.distance_traveled > 0.5:
            self.hatch.clear_to_retract = True
        if self.pursuit.completed_path and self.completed_runs > 3:
            self.next_state("stop")
        self.follow_path()
        if (
            not math.isnan(self.vision.target_tape_error) and self.ready_for_vision()
        ) or self.pursuit.completed_path:
            self.next_state("intake_hatch")

    @state
    def intake_hatch(self, initial_call):
        self.intake_hatch.engage()
        if not self.intake_hatch.is_executing:
            self.next_state("drive_to_loading_bay")

    @state
    def stop(self):
        self.chassis.set_inputs(0, 0, 0)
        self.done()

    @property
    def current_pos(self):
        return (self.chassis.odometry_x, self.chassis.odometry_y, self.imu.getAngle(), 2)

    def follow_path(self):
        vx, vy, heading = self.pursuit.find_velocity(self.current_pos)
        if self.pursuit.completed_path:
            self.chassis.set_inputs(0, 0, 0, field_oriented=False)
            return
        # TODO implement a system to allow for rotation in waypoints
        self.chassis.set_heading_sp(heading)
        self.chassis.set_inputs(vx, vy, 0)

    def ready_for_vision(self):
        if self.pursuit.waypoints[-1][4] - self.pursuit.distance_traveled < 1:
            return True
        else:
            return False


class RightStartAuto(AutoBase):
    MODE_NAME = "Right start autonomous"

    def __init__(self):
        super().__init__()
        # self.opp_front_cargo_bay = self.front_cargo_bay
        # self.opp_setup_loading_bay = self.setup_loading_bay
        self.opp_loading_bay = self.loading_bay
        self.opp_side_cargo_bay = self.side_cargo_bay
        self.opp_cross_point = self.cross_point

        self.front_cargo_bay = reflect_2d_y(self.front_cargo_bay)
        self.setup_loading_bay = reflect_2d_y(self.setup_loading_bay)
        self.loading_bay = reflect_2d_y(self.loading_bay)
        self.side_cargo_bay = reflect_2d_y(self.side_cargo_bay)
        self.cross_point = reflect_2d_y(self.cross_point)


class LeftStartAuto(AutoBase):
    MODE_NAME = "Left start autonomous"
    DEFAULT = True
