import math
import wpilib

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

    def __init__(self, angle_tolerance, range_tolerance):
        self.angle_tolerance = angle_tolerance
        self.range_tolerance = range_tolerance

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
        if (abs(self.vision.ground_tape_angle) > self.angle_tolerance) or (
            abs(self.vision.ground_tape_distance) > self.range_tolerance
        ):
            self.chassis.set_inputs(
                self.vision.ground_tape_distance
                * math.cos(self.vision.ground_tape_angle),
                self.vision.ground_tape_distance
                * math.sin(self.vision.ground_tape_angle),
                heading + self.vision.ground_tape_angle,
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
        if (abs(self.vision.target_tape_angle) > self.angle_tolerance) or (
            abs(self.vision.target_tape_distance) > self.range_tolerance
        ):
            self.chassis.set_inputs(
                self.vision.target_tape_distance
                * math.cos(self.vision.target_tape_angle),
                self.vision.target_tape_distance
                * math.sin(self.vision.target_tape_angle),
                heading + self.vision.ground_tape_angle,
            )
        else:
            self.next_state_now("ground_tape_align")
