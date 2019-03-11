from dataclasses import dataclass
import math

from magicbot.state_machine import AutonomousStateMachine, state
import wpilib

from automations.alignment import HatchDepositAligner, HatchIntakeAligner
from components.hatch import Hatch
from components.vision import Vision
from pyswervedrive.chassis import SwerveChassis
from utilities.navx import NavX
from utilities.pure_pursuit import PurePursuit, Waypoint, insert_trapezoidal_waypoints


@dataclass
class Coordinates:
    start_pos: Waypoint
    front_cargo_bay: Waypoint
    setup_loading_bay: Waypoint
    loading_bay: Waypoint
    side_cargo_bay_alignment_point: Waypoint
    side_cargo_bay: Waypoint


left_coordinates = Coordinates(
    start_pos=Waypoint(
        1.2 + SwerveChassis.LENGTH / 2, 0 + SwerveChassis.WIDTH / 2, 0, 2
    ),
    front_cargo_bay=Waypoint(5.5 - SwerveChassis.LENGTH / 2, 0.3, 0, 0.75),
    setup_loading_bay=Waypoint(3.3, 3.3, math.pi, 1),
    loading_bay=Waypoint(0.2 + SwerveChassis.LENGTH / 2, 3.4, math.pi, 1),
    side_cargo_bay_alignment_point=Waypoint(
        6.6, 1.8 + SwerveChassis.WIDTH / 2, -math.pi / 2, 0.75
    ),
    side_cargo_bay=Waypoint(6.6, 0.8 + SwerveChassis.WIDTH / 2, -math.pi / 2, 1),
)
right_coordinates = Coordinates(
    start_pos=left_coordinates.start_pos.reflect(),
    front_cargo_bay=left_coordinates.front_cargo_bay.reflect(),
    setup_loading_bay=left_coordinates.setup_loading_bay.reflect(),
    loading_bay=left_coordinates.loading_bay.reflect(),
    side_cargo_bay_alignment_point=left_coordinates.side_cargo_bay_alignment_point.reflect(),
    side_cargo_bay=left_coordinates.side_cargo_bay.reflect()
)


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
        self.coordinates: Coordinates
        self.coordinates = left_coordinates

        self.completed_runs = 0
        self.desired_angle = 0
        self.desired_angle_navx = 0
        self.minimum_path_completion = 0.85

        self.acceleration = 1
        self.deceleration = -0.5

        self.pursuit = PurePursuit(look_ahead=0.2, look_ahead_speed_modifier=0.25)

    def setup(self):
        self.hatch.has_hatch = True
        self.hatch_intake.alignment_speed = 1.2
        self.hatch_deposit.alignment_speed = 1.2

    def on_enable(self):
        super().on_enable()
        self.chassis.odometry_x = self.coordinates.start_pos.x
        self.chassis.odometry_y = self.coordinates.start_pos.y
        self.completed_runs = 0

    @state(first=True)
    def drive_to_cargo_bay(self, initial_call):
        if initial_call:
            if self.completed_runs == 0:
                waypoints = insert_trapezoidal_waypoints(
                    (self.current_pos, self.coordinates.front_cargo_bay),
                    self.acceleration,
                    self.deceleration,
                )
            elif self.completed_runs == 1:
                waypoints = insert_trapezoidal_waypoints(
                    (
                        self.current_pos,
                        self.coordinates.side_cargo_bay_alignment_point,
                        self.coordinates.side_cargo_bay,
                    ),
                    self.acceleration,
                    self.deceleration,
                )
            else:
                self.next_state("drive_to_loading_bay")
                self.completed_runs += 1
                return
            self.pursuit.build_path(waypoints)
        self.follow_path()
        if (
            self.vision.fiducial_in_sight and self.ready_for_vision()
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
                waypoints = insert_trapezoidal_waypoints(
                    (
                        self.current_pos,
                        Waypoint(
                            self.current_pos.x - 1,
                            self.current_pos.y,
                            self.imu.getAngle(),
                            1.5,
                        ),
                        self.coordinates.setup_loading_bay,
                        self.coordinates.loading_bay,
                    ),
                    self.acceleration,
                    self.deceleration,
                )
            elif self.completed_runs == 2:
                waypoints = insert_trapezoidal_waypoints(
                    (self.current_pos, self.coordinates.setup_loading_bay, self.coordinates.loading_bay),
                    self.acceleration,
                    self.deceleration,
                )
            else:
                self.next_state("stop")
                return
            self.pursuit.build_path(waypoints)
        self.follow_path()
        if (
            self.vision.fiducial_in_sight and self.ready_for_vision()
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
        return Waypoint(
            self.chassis.odometry_x, self.chassis.odometry_y, self.imu.getAngle(), 2
        )

    def follow_path(self):
        vx, vy, heading = self.pursuit.find_velocity(self.chassis.position)
        if self.pursuit.completed_path:
            self.chassis.set_inputs(0, 0, 0, field_oriented=True)
            return
        self.chassis.set_velocity_heading(vx, vy, heading)

    def ready_for_vision(self):
        if self.pursuit.waypoints[-1][4] - self.pursuit.distance_traveled < 1:
            return True
        else:
            return False


class RightFullAuto(AutoBase):
    MODE_NAME = "Right Full Autonomous"

    def __init__(self):
        super().__init__()
        self.coordinates = right_coordinates


class LeftFullAuto(AutoBase):
    MODE_NAME = "Left Full Autonomous"


class FrontOnlyBase(AutoBase):
    @state
    def deposit_hatch(self, initial_call):
        if initial_call:
            self.hatch_deposit.engage(initial_state="target_tape_align")
        if not self.hatch.has_hatch:
            self.done()


class LeftFrontOnly(FrontOnlyBase):
    MODE_NAME = "Left Front Hatch Only"


class RightFrontOnly(FrontOnlyBase):
    MODE_NAME = "Right Front Hatch Only"

    def __init__(self):
        super().__init__()
        self.coordinates = right_coordinates


class SideOnlyBase(AutoBase):
    @state(first=True)
    def drive_to_cargo_bay(self, initial_call):
        if initial_call:
            waypoints = insert_trapezoidal_waypoints(
                (
                    self.current_pos,
                    self.coordinates.side_cargo_bay_alignment_point,
                    self.coordinates.side_cargo_bay,
                ),
                self.acceleration,
                self.deceleration,
            )
            self.pursuit.build_path(waypoints)
        self.follow_path()
        if (
            self.vision.fiducial_in_sight and self.ready_for_vision()
        ) or self.pursuit.completed_path:
            self.next_state("deposit_hatch")

    @state
    def deposit_hatch(self, initial_call):
        if initial_call:
            self.hatch_deposit.engage(initial_state="target_tape_align")
        if not self.hatch.has_hatch:
            self.done()


class LeftSideOnly(SideOnlyBase):
    MODE_NAME = "Left Side Hatch Only"


class RightSideOnly(SideOnlyBase):
    MODE_NAME = "Right Side Hatch Only"

    def __init__(self):
        super().__init__()
        self.coordinates = right_coordinates
