import math
import wpilib
import numpy as np

from vision import Vision
from magicbot.state_machine import StateMachine, state
from pyswervedrive.chassis import Chassis
from utilities.imu import IMU


class AlignmentStateMachine(StateMachine):
    """
    A state machine for alignment using vision systems.

    The robot will use two methods of alignment, targets above
    objectives from longer range and fine adjustment using the ground
    tape once we are able to see it.
    """

    vision: Vision
    chassis: Chassis
    imu: IMU

    def __init__(
        self,
        angle_tolerance,
        closing_step_size,
        max_target_tape_loops,
        approach_offset_angle,
    ):
        self.angle_tolerance = angle_tolerance
        self.closing_step_size = closing_step_size
        self.target_tape_loops = 0
        self.max_target_tape_loops = max_target_tape_loops
        self.approach_offset_angle = approach_offset_angle
        self.close_speed_denominator = 4
        self.close_side_speed_denominator = 10

    @state(first=True)
    def check_ground(self):
        """Check for vision tape on the ground"""
        if self.vision.ground_tape_angle == []:
            self.next_state_now("ground_align")
        else:
            self.next_state_now("target_tape_align")

    @state
    def ground_tape_align(self):
        """
        Align with the objective by way of the ground vision tape.

        The robot will attempt to correct errors untill they are within
        tolerance, commands a constant forwards velocity with lateral
        corrections based off the difference between the vision angle and
        angle measured by the IMU.
        NOTE: the set_inputs command could exceed the limitations of the
        hardware, this is handled by the drivebase control system
        """
        heading = self.imu.getAngle()
        if abs(self.vision.ground_tape_angle) > self.angle_tolerance:
            vision_vector = np.array(
                math.cos(self.vision.ground_tape_angle),
                math.sin(self.vision.ground_tape_angle),
            )
            if self.vision.ground_tape_angle - heading > 0:
                rotated_vision_vector = self.rotate(vision_vector, 90)
            elif self.vision.ground_tape_angle - heading < 0:
                rotated_vision_vector = self.rotate(vision_vector, -90)
            else:
                rotated_vision_vector = vision_vector
            self.chassis.set_inputs(
                vision_vector[0] / self.close_speed_denominator
                + rotated_vision_vector[0] / self.close_side_speed_denominator,
                vision_vector[1] / self.close_speed_denominator
                + rotated_vision_vector[1] / self.close_side_speed_denominator,
                heading + self.vision.ground_tape_angle,
                # TODO tune these magic numbers
            )
        else:
            self.done()

    @state
    def target_tape_align(self):
        """
        Align with the objective using the vision tape above the objective.

        The robot will try to correct errors untill they are within tolerance
        by moving and rotating towards the measured vision angle, once
        successful move on to the ground alignment system.
        NOTE: the set_inputs command could exceed the limitations of the
        hardware, this is handled by the drivebase control system
        """
        heading = self.imu.getAngle()
        if abs(self.vision.target_tape_angle) > self.angle_tolerance:
            vision_vector = np.array(
                math.cos(self.vision.target_tape_angle),
                math.sin(self.vision.target_tape_angle),
            )
            rotated_vision_vector = self.rotate(
                vision_vector, self.approach_offset_angle
            )
            self.chassis.set_inputs(
                rotated_vision_vector[0],
                rotated_vision_vector[1],
                heading + self.vision.target_tape_angle,
            )
        elif self.target_tape_loops >= self.max_target_tape_loops:
            self.next_state_now("move_closer")
        else:
            self.target_tape_loops += 1
            self.next_state_now("check_ground")

    @state
    def move_closer(self, initial_call):
        """
        Drive towards the target to try to find the ground tape.

        After the robot has failed to spot the ground tape, move forwards
        transition back to the check ground state
        """
        if initial_call:
            starting_odometry = self.chassis.odometry
        heading = self.imu.getAngle()
        self.chassis.set_inputs(
            math.cos(self.vision.target_tape_angle),
            math.sin(self.vision.target_tape_angle),
            heading + self.vision.ground_tape_angle,
        )
        if self.chassis.odometry - starting_odometry > self.closing_step_size:
            self.next_state_now("check_ground")

    def rotate(self, A, angle):
        rotation_matrix = np.array(
            [[math.cos(angle), math.sin(angle)], [-math.sin(angle), math.cos(angle)]]
        )
        return A @ rotation_matrix
