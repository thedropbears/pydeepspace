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

    kP_x = tunable(0.5)  # forwards
    kP_y = tunable(1)  # m/s
    tolerance = tunable(0.05)  # % of camera view

    @state(first=True)
    def target_tape_align(self, initial_call):
        """
        Align with the objective using the vision tape above the objective.

        The robot will try to correct errors untill they are within tolerance
        by strafing and moving in a hyberbolic curve towards the target.
        """
        kP_x = self.kP_x
        kP_y = self.kP_y
        tolerance = self.tolerance
        if initial_call:
            self.loop_counter = 0
        error = self.vision.get_target_tape_error()
        if error is None:
            self.loop_counter += 1
            if self.loop_counter > 3:
                self.chassis.set_inputs(0, 0, 0)
                self.done()
                return
        else:
            if abs(error) > tolerance:
                self.chassis.set_inputs(
                    (1 - abs(error)) * kP_x, error * kP_y, 0, field_oriented=False
                )
            else:
                self.done()
