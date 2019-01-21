import math
import numpy as np
from pyswervedrive.swervechassis import SwerveChassis


class PurePursuit:
    """
    Pure Pursuit controller for navigation with absolute waypoints

    Uses the method outlined here
    https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf
    """
    chassis: SwerveChassis

    def __init__(self, look_ahead, speed_modifier):
        self.segments = None
        self.look_ahead = look_ahead
        self.current_segment = None
        self.speed_modifier = speed_modifier

    def find_intersections(self, segment):
        """
        Find the intersection/s between our lookahead distance and path.

        http://mathworld.wolfram.com/Circle-LineIntersection.html
        NOTE: this will return the intersections in global co-ordinates
        """
        x_1, y_1 = segment[0], segment[1]
        x_2, y_2 = segment[2], segment[3]
        goal_point = np.array((x_2, y_2))

        d_x = x_2 - x_1
        d_y = y_2 - y_1
        d_r = math.sqrt(d_x ** 2 + d_y ** 2)
        D = np.array((x_1, y_1), (x_2, y_2))
        r = self.look_ahead
        discriminent = r ** 2 * d_r ** 2 - D ** 2

        if discriminent >= 0:  # if an intersection exists
            intersection_1 = np.zeros((2, 1))
            intersection_2 = np.zeros((2, 1))
            sqrt_discriminent = math.sqrt(discriminent)

            right_x = self.sgn(d_y) * d_x * sqrt_discriminent
            left_x = D * d_y
            right_y = abs(d_y) * sqrt_discriminent
            left_y = -1 * D * d_x
            denominator = d_r ** 2

            intersection_1[0] = (left_x + right_x) / denominator
            intersection_1[1] = (left_y + right_y) / denominator
            if discriminent == 0:  # if we are tangent to our path
                return intersection_1
            intersection_2[0] = (left_x - right_x) / denominator
            intersection_2[1] = (left_y - right_y) / denominator
            if abs(intersection_1) - abs(goal_point) < abs(intersection_2) - abs(goal_point):
                return intersection_1
            else:
                return intersection_2

    def build_path(self, waypoints):
        # check if we actually need to create seperate segments or if we can simply use the waypoints
        self.segments = waypoints

    def follow_path(self):
        goal_point = self.find_intersections(self.current_segment)
        goal_point -= self.chassis.position()
        goal_point = self.chassis.robot_orient(goal_point)
        self.chassis.set_inputs(*goal_point*self.speed_modifier, 0)
        # need to find a better way to scale the input to the chassis

    def sgn(self, number):
        """Returns the sign of a number, 0 is positive"""
        if number < 0:
            return -1
        else:
            return 1
