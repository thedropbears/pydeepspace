from dataclasses import dataclass
import math

from magicbot.state_machine import AutonomousStateMachine, state
import wpilib

from automations.alignment import (
    HatchDepositAligner,
    HatchIntakeAligner,
    CargoDepositAligner,
)
from automations.cargo import CargoManager
from components.cargo import CargoManipulator
from components.hatch import Hatch
from components.vision import Vision
from pyswervedrive.chassis import SwerveChassis
from utilities.navx import NavX
from utilities.pure_pursuit import PurePursuit, Waypoint, insert_trapezoidal_waypoints


@dataclass
class Coordinates:
    start_pos: Waypoint
    start_pos_cargo: Waypoint
    front_cargo_ship: Waypoint
    setup_loading_bay: Waypoint
    loading_bay: Waypoint
    cargo_depot_setup: Waypoint
    cargo_depot: Waypoint
    side_cargo_ship_alignment_point: Waypoint
    side_cargo_ship_middle: Waypoint
    side_cargo_ship: Waypoint
    cargo_endpoint: Waypoint


left_coordinates = Coordinates(
    start_pos=Waypoint(
        1.3 + SwerveChassis.LENGTH / 2, 0 + SwerveChassis.WIDTH / 2, 0, 2
    ),
    start_pos_cargo=Waypoint(
        1.3 + SwerveChassis.LENGTH / 2, 1.63 - SwerveChassis.WIDTH / 2, 0, 2
    ),
    front_cargo_ship=Waypoint(5.5 - SwerveChassis.LENGTH / 2, 0.3, 0, 1.5),
    setup_loading_bay=Waypoint(3.3, 3, math.pi, 2),
    loading_bay=Waypoint(0.2 + SwerveChassis.LENGTH / 2, 3.4, math.pi, 1.5),
    cargo_depot_setup=Waypoint(2.4, 3, -math.atan2(2.4 - 1.2, 3 - 2.3), 1),
    cargo_depot=Waypoint(1.2, 2.3, -math.atan2(2.4 - 1.2, 3 - 2.3), 0.5),
    side_cargo_ship_alignment_point=Waypoint(
        6.6, 1.8 + SwerveChassis.WIDTH / 2, -math.pi / 2, 1.5
    ),
    side_cargo_ship_middle=Waypoint(
        (6.6 + 7.4) / 2, 0.8 + SwerveChassis.WIDTH / 2, math.pi / 2, 0.75
    ),
    side_cargo_ship=Waypoint(6.6, 0.8 + SwerveChassis.WIDTH / 2, -math.pi / 2, 1),
    cargo_endpoint=Waypoint(7.4, 1.3, 0, 3),
)
right_coordinates = Coordinates(
    start_pos=left_coordinates.start_pos.reflect(),
    start_pos_cargo=left_coordinates.start_pos_cargo.reflect(),
    front_cargo_ship=left_coordinates.front_cargo_ship.reflect(),
    setup_loading_bay=left_coordinates.setup_loading_bay.reflect(),
    loading_bay=left_coordinates.loading_bay.reflect(),
    cargo_depot_setup=left_coordinates.cargo_depot_setup.reflect(),
    cargo_depot=left_coordinates.cargo_depot.reflect(),
    side_cargo_ship_alignment_point=left_coordinates.side_cargo_ship_alignment_point.reflect(),
    side_cargo_ship_middle=left_coordinates.side_cargo_ship_middle.reflect(),
    side_cargo_ship=left_coordinates.side_cargo_ship.reflect(),
    cargo_endpoint=left_coordinates.cargo_endpoint.reflect(),
)


class AutoBase(AutonomousStateMachine):

    # Here magicbot injects components
    hatch_deposit: HatchDepositAligner
    hatch_intake: HatchIntakeAligner
    cargo: CargoManager

    chassis: SwerveChassis
    hatch: Hatch
    cargo_component: CargoManipulator
    imu: NavX
    vision: Vision

    # This one is just a typehint
    pursuit: PurePursuit

    def __init__(self):
        super().__init__()
        self.coordinates: Coordinates = left_coordinates

        self.completed_runs = 0
        self.desired_angle = 0
        self.desired_angle_navx = 0
        self.minimum_path_completion = 0.85

        self.pursuit = PurePursuit(look_ahead=0.2, look_ahead_speed_modifier=0.25)

    def setup(self):
        self.hatch.has_hatch = True
        self.vision.use_hatch()

    def on_enable(self):
        super().on_enable()
        self.chassis.odometry_x = self.coordinates.start_pos.x
        self.chassis.odometry_y = self.coordinates.start_pos.y
        self.completed_runs = 0

    # @state(first=True)
    # def intake_starting_hatch(self, initial_call):
    #     if initial_call:
    #         self.counter = 0
    #     if self.counter > 5:
    #         self.next_state("drive_to_cargo_ship")
    #     else:
    #         self.counter += 1

    @state(first=True)
    def drive_to_cargo_ship(self, initial_call):
        if initial_call:
            if self.completed_runs == 0:
                waypoints = insert_trapezoidal_waypoints(
                    (self.current_pos, self.coordinates.front_cargo_ship),
                    self.chassis.acceleration,
                    self.chassis.deceleration,
                )
            elif self.completed_runs == 1:
                waypoints = insert_trapezoidal_waypoints(
                    (
                        self.current_pos,
                        self.coordinates.side_cargo_ship_alignment_point,
                        self.coordinates.side_cargo_ship,
                    ),
                    self.chassis.acceleration,
                    self.chassis.deceleration,
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
                    self.chassis.acceleration,
                    self.chassis.deceleration,
                )
            elif self.completed_runs == 2:
                waypoints = insert_trapezoidal_waypoints(
                    (
                        self.current_pos,
                        self.coordinates.setup_loading_bay,
                        self.coordinates.loading_bay,
                    ),
                    self.chassis.acceleration,
                    self.chassis.deceleration,
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
            self.next_state("drive_to_cargo_ship")

    @state
    def stop(self):
        self.chassis.set_inputs(0, 0, 0)
        self.done()

    @property
    def current_pos(self):
        return Waypoint(
            self.chassis.odometry_x, self.chassis.odometry_y, self.imu.getAngle(), 3
        )

    def follow_path(self):
        vx, vy, heading = self.pursuit.find_velocity(self.chassis.position)
        if self.pursuit.completed_path:
            self.chassis.set_inputs(0, 0, 0, field_oriented=True)
            return
        self.chassis.set_velocity_heading(vx, vy, heading)

    def ready_for_vision(self):
        if self.pursuit.waypoints[-1][4] - self.pursuit.distance_traveled < 2:
            return True
        else:
            return False


class RightFullAuto(AutoBase):
    MODE_NAME = "Right Full Autonomous"
    DEFAULT = True

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
    def drive_to_cargo_ship(self, initial_call):
        if initial_call:
            waypoints = insert_trapezoidal_waypoints(
                (
                    self.current_pos,
                    self.coordinates.side_cargo_ship_alignment_point,
                    self.coordinates.side_cargo_ship,
                ),
                self.chassis.acceleration,
                self.chassis.deceleration,
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


class DriveForwards(AutonomousStateMachine):
    MODE_NAME = "Drive Forwards - Default"

    chassis: SwerveChassis
    imu: NavX
    hatch: Hatch
    cargo_component: CargoManipulator

    joystick: wpilib.Joystick

    def __init__(self):
        super().__init__()
        self.pursuit = PurePursuit(look_ahead=0.2, look_ahead_speed_modifier=0.25)

    def on_enable(self):
        super().on_enable()
        self.chassis.odometry_x = 0
        self.chassis.odometry_y = 0

    @state(first=True)
    def wait_for_input(self):
        self.hatch.has_hatch = False
        self.cargo_component.has_cargo = True
        if self.joystick.getY() < -0.5:  # joystick -y is forwards
            self.next_state("drive_forwards")

    @state
    def drive_forwards(self, initial_call):
        if initial_call:
            waypoints = insert_trapezoidal_waypoints(
                (self.current_pos, Waypoint(1.5, 0, 0, 0)),
                acceleration=self.chassis.acceleration,
                deceleration=self.chassis.deceleration,
            )
            self.pursuit.build_path(waypoints)
        self.follow_path()
        if self.pursuit.completed_path:
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


class DoubleFrontBase(AutoBase):
    @state(first=True)
    def drive_to_cargo_ship(self, initial_call):
        if initial_call:
            if self.completed_runs == 0:
                waypoints = insert_trapezoidal_waypoints(
                    (self.current_pos, self.coordinates.front_cargo_ship),
                    self.chassis.acceleration,
                    self.chassis.deceleration,
                )
            elif self.completed_runs == 1:
                waypoints = insert_trapezoidal_waypoints(
                    (
                        self.current_pos,
                        self.coordinates.setup_loading_bay,
                        self.coordinates.front_cargo_ship.reflect(),
                    ),
                    self.chassis.acceleration,
                    self.chassis.deceleration,
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


class LeftDoubleFront(DoubleFrontBase):
    MODE_NAME = "Left Double Front Hatch"


class RightDoubleFront(DoubleFrontBase):
    MODE_NAME = "Right Double Front Hatch"

    def __init__(self):
        super().__init__()
        self.coordinates = right_coordinates


class CargoAutoBase(AutoBase):

    vision: Vision

    cargo_deposit: CargoDepositAligner

    cargo_component: CargoManipulator

    def __init__(self):
        super().__init__()
        self.cargo_intake_speed = 0.5

    def setup(self):
        self.cargo.has_cargo = True
        self.vision.use_cargo()

    def on_enable(self):
        super().on_enable()
        self.chassis.odometry_x = self.coordinates.start_pos_cargo.x
        self.chassis.odometry_y = self.coordinates.start_pos_cargo.y

    @state(first=True)
    def drive_to_cargo_ship(self, initial_call):
        if initial_call:
            self.hatch.has_hatch = False
            self.cargo_component.has_cargo = True
            waypoints = insert_trapezoidal_waypoints(
                (
                    self.current_pos,
                    self.coordinates.side_cargo_ship_alignment_point,
                    self.coordinates.side_cargo_ship_middle,
                ),
                self.chassis.acceleration,
                self.chassis.deceleration,
            )
            self.pursuit.build_path(waypoints)
        self.follow_path()
        if (
            self.vision.fiducial_in_sight and self.ready_for_vision()
        ) or self.pursuit.completed_path:
            self.next_state("deposit_cargo")

    @state
    def deposit_cargo(self, initial_call):
        if initial_call:
            self.cargo_deposit.engage(initial_state="target_tape_align")
        if not (self.cargo_deposit.is_executing or self.cargo.is_executing):
            self.next_state("drive_to_cargo_depot_setup")

    @state
    def drive_to_cargo_depot_setup(self, initial_call):
        if initial_call:
            waypoints = insert_trapezoidal_waypoints(
                (self.current_pos, self.coordinates.cargo_depot_setup),
                self.chassis.acceleration,
                self.chassis.deceleration,
            )
            self.pursuit.build_path(waypoints)
        self.follow_path()
        if self.pursuit.completed_path:
            self.done()
            # self.next_state("intake_cargo")

    # @state
    # def intake_cargo(self, initial_call):
    #     """
    #     Start cargo intake and move forwards slowly
    #     """
    #     if initial_call:
    #         self.cargo.intake_floor()
    #         self.chassis.set_inputs(
    #             -self.cargo_intake_speed, 0, 0, field_orineted=False
    #         )
    #         # Move towards the cargo side of the robot
    #     if self.cargo_component.has_cargo:
    #         self.next_state("drive_to_endpoint")

    @state
    def drive_to_endpoint(self, initial_call):
        """
        Move to the point where we hand over to drivers
        """
        if initial_call:
            waypoints = insert_trapezoidal_waypoints(
                (
                    self.current_pos,
                    self.coordinates.side_cargo_ship_alignment_point,
                    self.coordinates.cargo_endpoint,
                ),
                self.chassis.acceleration,
                self.chassis.deceleration,
            )
            self.pursuit.build_path(waypoints)
        self.follow_path()
        if self.pursuit.completed_path:
            self.done()


class RightCargoAuto(CargoAutoBase):
    MODE_NAME = "Right Cargo Pickup"

    def __init__(self):
        super().__init__()
        self.coordinates = right_coordinates


class LeftCargoAuto(CargoAutoBase):
    MODE_NAME = "Left Cargo Pickup"

    def __init__(self):
        super().__init__()
        self.coordinates = left_coordinates
