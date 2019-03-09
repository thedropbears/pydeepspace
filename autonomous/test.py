from magicbot.state_machine import AutonomousStateMachine, state

from pyswervedrive.chassis import SwerveChassis
from utilities.navx import NavX
from utilities.pure_pursuit import PurePursuit, Waypoint


class TestPursuitAuto(AutonomousStateMachine):

    MODE_NAME = "Test Pursuit Auto"
    DEFAULT = False

    chassis: SwerveChassis
    imu: NavX

    def on_enable(self):
        super().on_enable()
        self.chassis.odometry_x = 0
        self.chassis.odometry_y = 0
        self.points = (
            self.current_pos,
            Waypoint(2, 0, 0, 1),
            Waypoint(2, 2, 0, 1),
            Waypoint(0, 2, 0, 1),
        )
        self.pursuit = PurePursuit(look_ahead=0.2, look_ahead_speed_modifier=0.0)

    @state(first=True)
    def move_forwards(self, initial_call):
        if initial_call:
            self.pursuit.build_path(self.points)
        vx, vy, vz = self.pursuit.find_velocity(self.chassis.position)
        self.chassis.set_inputs(vx, vy, 0)
        if self.pursuit.completed_path:
            self.chassis.set_inputs(0, 0, 0)
            self.done()

    @property
    def current_pos(self):
        return Waypoint(
            self.chassis.odometry_x, self.chassis.odometry_y, self.imu.getAngle(), 1
        )
