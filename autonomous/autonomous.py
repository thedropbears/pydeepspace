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
        self.front_cargo_bay = (5.6 - SwerveChassis.LENGTH / 2, 0.2, 0, 1)
        self.front_cargo_bay_slowdown = (4.6 - SwerveChassis.LENGTH / 2, 0.2, 0, 2)
        self.setup_loading_bay = (3.3, 3.3, math.pi, 2)
        self.loading_bay = (0.2 + SwerveChassis.LENGTH / 2, 3.4, math.pi, 1)
        self.loading_bay_speedup_point = (1 + SwerveChassis.LENGTH / 2, 3.4, math.pi, 2)
        self.side_cargo_bay = (7, 0.8 + SwerveChassis.WIDTH / 2, -math.pi / 2, 1)
        self.side_cargo_bay_slowdown_point = (
            7,
            1.8 + SwerveChassis.WIDTH / 2,
            -math.pi / 2,
            2,
        )
        self.start_pos = (
            1.2 + SwerveChassis.LENGTH / 2,
            0,
            0,
            2,
            #  + SwerveChassis.WIDTH / 2
        )

        self.completed_runs = 0
        self.desired_angle = 0
        self.desired_angle_navx = 0
        self.minimum_path_completion = 0.85

        self.pursuit = PurePursuit(look_ahead=0.2, look_ahead_speed_modifier=0.25)

    def on_enable(self):
        super().on_enable()
        self.chassis.odometry_x = self.start_pos[0]
        self.chassis.odometry_y = self.start_pos[1]
        self.completed_runs = 0
        # print(f"odometry = {self.current_pos}")

    @state(first=True)
    def drive_to_cargo_bay(self, initial_call):
        if initial_call:
            # print(f"odometry = {self.current_pos}")
            if self.completed_runs == 0:
                self.pursuit.build_path(
                    (
                        self.current_pos,
                        self.front_cargo_bay_slowdown,
                        self.front_cargo_bay,
                    )
                )
            elif self.completed_runs == 1:
                self.pursuit.build_path(
                    (
                        self.current_pos,
                        self.loading_bay_speedup_point,
                        self.side_cargo_bay_slowdown_point,
                        self.side_cargo_bay,
                    )
                )
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
        if initial_call:
            self.hatch_deposit.engage(initial_state="target_tape_align")
        if not self.hatch.has_hatch:
            self.next_state("drive_to_loading_bay")

    @state
    def drive_to_loading_bay(self, initial_call):
        if initial_call:
            if self.completed_runs == 1:
                self.pursuit.build_path(
                    (
                        self.current_pos,
                        (
                            self.current_pos[0] - 0.5,
                            self.current_pos[1],
                            self.imu.getAngle(),
                            1.5,
                        ),
                        self.setup_loading_bay,
                        self.loading_bay,
                    )
                )
            elif self.completed_runs == 2:
                self.pursuit.build_path(
                    (self.current_pos, self.setup_loading_bay, self.loading_bay)
                )
            elif self.completed_runs == 3:
                self.next_state("stop")
        self.follow_path()
        if (
            not math.isnan(self.vision.target_tape_error) and self.ready_for_vision()
        ) or self.pursuit.completed_path:
            self.next_state("intake_hatch")

    @state
    def intake_hatch(self, initial_call):
        if initial_call:
            self.hatch_intake.engage(initial_state="target_tape_align")
        elif not self.hatch_intake.is_executing:
            self.next_state("drive_to_cargo_bay")

    @state
    def stop(self):
        self.chassis.set_inputs(0, 0, 0)
        self.done()

    @property
    def current_pos(self):
        return (
            self.chassis.odometry_x,
            self.chassis.odometry_y,
            self.imu.getAngle(),
            2,
        )

    def follow_path(self):
        vx, vy, heading = self.pursuit.find_velocity(self.current_pos)
        if self.pursuit.completed_path:
            self.chassis.set_inputs(0, 0, 0, field_oriented=False)
            return
        # TODO implement a system to allow for rotation in waypoints
        self.chassis.set_heading_sp(heading)
        self.chassis.set_inputs(vx, vy, 0)

    def ready_for_vision(self):
        if (self.pursuit.waypoints[-1][4] - self.pursuit.distance_traveled < 1) and (
            self.pursuit.current_waypoint_number > len(self.pursuit.waypoints) - 1
        ):
            return True
        else:
            return False


class RightStartAuto(AutoBase):
    MODE_NAME = "Right start autonomous"

    def __init__(self):
        super().__init__()
        self.front_cargo_bay = reflect_2d_y(self.front_cargo_bay)
        self.setup_loading_bay = reflect_2d_y(self.setup_loading_bay)
        self.loading_bay = reflect_2d_y(self.loading_bay)
        self.side_cargo_bay = reflect_2d_y(self.side_cargo_bay)


class LeftStartAuto(AutoBase):
    MODE_NAME = "Left start autonomous"
    DEFAULT = True
