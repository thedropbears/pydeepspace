from magicbot.state_machine import AutonomousStateMachine, state
from utilities.pure_pursuit import PurePursuit
from pyswervedrive.chassis import SwerveChassis
from utilities.navx import NavX


class TestPursuitAuto(AutonomousStateMachine):

    MODE_NAME = "Test Pursuit Auto"
    DEFAULT = False

    chassis: SwerveChassis
    pursuit: PurePursuit
    imu: NavX

    def __init__(self):
        super().__init__()
        self.points = ((0, 0, 0, 1), (0.8, 0, 0, 0.5), (1, 0, 0, 0.2))
        self.pursuit = PurePursuit(look_ahead=0.2, look_ahead_speed_modifier=0.25)

    @state(first=True)
    def move_forwards(self, initial_call):
        if initial_call:
            self.pursuit.build_path(self.points)
        heading = self.imu.getAngle()
        x, y = self.chassis.position
        position = (x, y, heading)
        vx, vy, vz = self.pursuit.find_velocity(position)
        self.chassis.set_inputs(vx, vy, 0)
        if self.pursuit.completed_path:
            self.chassis.set_inputs(0, 0, 0)
            self.done()
