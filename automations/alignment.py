import math

from components.vision import Vision
from magicbot.state_machine import StateMachine, state
from pyswervedrive.swervechassis import SwerveChassis
from magicbot import tunable


class Aligner(StateMachine):
    """
    A state machine for alignment using vision systems.

    The robot will use two methods of alignment, targets above
    objectives from longer range and fine adjustment using the ground
    tape once we are able to see it.
    """

    chassis: SwerveChassis
    vision: Vision

    def setup(self):
        self.loop_counter = 0
        self.successful = False

    target_tape_kP_x = tunable(0.5)  # forwards
    target_tape_kP_y = tunable(1)  # m/s
    target_tape_tolerance = tunable(0.05)  # % of camera view

    ground_tape_kP_y = tunable(1)  # m/s
    ground_tape_kP_angle = tunable(2)
    ground_tape_distance_tolerance_y = tunable(0.02)
    ground_tape_angle_tolerance = tunable(math.pi / 360)  # this equals 0.5 degres

    @state(first=True)
    def target_tape_align(self, initial_call):
        """
        Align with the objective using the vision tape above the objective.

        The robot will try to correct errors untill they are within tolerance
        by strafing and moving in a hyberbolic curve towards the target.
        """
        if initial_call:
            self.loop_counter = 0
        error = self.vision.get_target_tape_error()
        if error is None:
            self.loop_counter += 1
            if self.loop_counter > 3:
                self.chassis.set_inputs(0, 0, 0)
                self.successful = False
                self.done()
                return
        if self.vision.get_ground_tape_y is not None:
            self.next_state_now("ground_tape_align")
        if abs(error) > self.target_tape_tolerance:
            vy = error * self.target_tape_kP_y
        else:
            vy = 0
        vx = (1 - abs(error)) * self.target_tape_kP_x
        self.chassis.set_inputs(vx, vy, 0, field_oriented=False)

    @state
    def ground_tape_align(self, initial_call):
        if initial_call:
            self.loop_counter = 0
        error_y = self.vision.get_ground_tape_y
        error_angle = self.vision.get_ground_tape_angle()
        if error_y is None:
            self.loop_counter += 1
            if self.loop_counter > 3:
                self.chassis.set_inputs(0, 0, 0)
                self.successful = False
                self.done()
        else:
            if abs(error_angle) <= self.ground_tape_angle_tolerance:
                error_angle = 0
            else:
                self.chassis.set_inputs(0, 0, error_angle * self.ground_tape_kP_angle)
            if abs(error_y) <= self.ground_tape_distance_tolerance_y:
                error_y = 0
            else:
                self.chassis.set_inputs(0, 0, error_y * self.ground_tape_kP_y)
            if self.vision.within_deposit_range:
                vx = 0
            else:
                vx = 0.5
            if vx and error_y and error_angle == 0:
                self.successful = True
                self.done()
