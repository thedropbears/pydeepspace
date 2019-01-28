from components.vision import Vision
from magicbot.state_machine import StateMachine, state
from pyswervedrive.chassis import SwerveChassis
from magicbot import tunable


class Aligner(StateMachine):
    """
    A state machine for alignment using vision systems.

    The robot will use two methods of alignment, targets above
    objectives from longer range and fine adjustment using the ground
    tape once we are able to see it.
    """

    VERBOSE_LOGGING = True

    chassis: SwerveChassis
    vision: Vision

    def setup(self):
        self.loop_counter = 0
        self.successful = False
        self.hatch_deposit = 0
        self.hatch_intake = 1
        self.cargo_outake = 2
        self.mode = self.hatch_deposit

    target_tape_kP_x = tunable(0.5)  # forwards
    target_tape_kP_y = tunable(1)  # m/s
    target_tape_tolerance = tunable(0.05)  # % of camera view

    @state(first=True)
    def target_tape_align(self, initial_call, state_tm):
        """
        Align with the objective using the vision tape above the objective.

        The robot will try to correct errors untill they are within tolerance
        by strafing and moving in a hyberbolic curve towards the target.
        """
        self.chassis.heading_hold_off()
        if initial_call:
            self.loop_counter = 0
            self.successful = False
        error = self.vision.get_target_tape_error()
        if error is None:
            self.chassis.set_inputs(0, -0.5, 0, field_oriented=False)
            if state_tm > 2.5:
                self.successful = True
                self.chassis.set_inputs(0, 0, 0)
                self.done()
        else:
            vy = error * self.target_tape_kP_y
            vx = (1 - abs(error)) * self.target_tape_kP_x
            self.chassis.set_inputs(vy, -vx, 0, field_oriented=False)
            # Rotate our co-ordinate system because the hatch mechanism is
            # on the 'side' of the robot
            if self.vision.within_deposit_range:
                self.next_state("success")

    @state
    def success(self, state_tm):
        self.chassis.set_inputs(0, -0.5, 0, field_oriented=False)
        if state_tm > 0.5:
            self.successful = True
            self.chassis.set_inputs(0, 0, 0)
            self.done()
