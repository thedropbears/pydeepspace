import math
import numpy as np
from magicbot.state_machine import AutonomousStateMachine, state
from utilities.pure_pursuit import PurePursuit
from pyswervedrive.swervechassis import SwerveChassis
from utilities.navx import NavX


class TestPursuitAuto(AutonomousStateMachine):

    MODE_NAME = "Test Pursuit Auto"
    DEFAULT = True

    chassis: SwerveChassis
    pursuit: PurePursuit
    imu: NavX

    Kp = 4
    Ki = 0.1
    Kd = 0

    def __init__(self):
        super().__init__()
        self.points = np.array(((0.01, -0.01), (2, -1.7), (4, -6), (2, 3)))
        self.loops = 1
        self.error_i = 0
        self.last_error = 0

    @state(first=True)
    def move_forwards_and_back(self, initial_call):
        if initial_call:
            self.pursuit.build_path(self.points)
            self.loops = 1
        heading = self.imu.getAngle()
        x, y = self.chassis.position
        position = np.array((x[0], y[0], heading))
        vector, changed_waypoint = self.pursuit.compute_direction(position)
        vx, vy = vector[0], vector[1]
        if changed_waypoint:
            self.error_i = 0
            self.last_error = 0
        # vector = np.array((vx, vy))
        # normalised = vector / np.linalg.norm(vector)
        # self.chassis.set_inputs(normalised[0]/1, normalised[1]/1, 0)
        error = np.linalg.norm(position[:2] - self.pursuit.waypoints[self.pursuit.current_waypoint_number+1])
        self.error_i += error
        error_d = error - self.last_error
        output = error*self.Kp+self.Ki/self.error_i+self.Kd*(error_d/20)
        print(output)
        self.chassis.set_inputs(output*vx, output*vy, 0)
        self.loops += 1
        last_error = error
