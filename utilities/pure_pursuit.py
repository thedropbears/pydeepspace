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

    def __init__(self, look_ahead, speed_modifier, ending_tolerance):
        self.waypoints = []
        self.look_ahead = look_ahead
        self.speed_modifier = speed_modifier
        self.ending_tolerance = ending_tolerance
        self.current_waypoint_number = 0

    def find_intersections(self, waypoint_start, waypoint_end):
        """
        Find the intersection/s between our lookahead distance and path.

        http://mathworld.wolfram.com/Circle-LineIntersection.html
        NOTE: this will return the intersections in global co-ordinates
        """
        x_1, y_1 = waypoint_start[0], waypoint_start[1]
        x_2, y_2 = waypoint_end[2], waypoint_end[3]
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
        self.waypoints = waypoints

    def follow_path(self):
        """Find the goal_point and convert it to relative co-ordinates"""
        goal_point = self.find_intersections(self.waypoints[self.current_waypoint_number], self.waypoints[self.current_waypoint_number])
        goal_point -= self.chassis.position()
        goal_point = self.chassis.robot_orient(goal_point)
        goal_point = goal_point / np.linalg.norm(goal_point)
        self.check_progress()
        self.chassis.set_inputs(*goal_point*self.speed_modifier, 0)
        # need to find a better way to scale the input to the chassis

    def check_progress(self, segment):
        """Check if we are close enough to begin the path to the next waypoint"""
        end_point_x, end_point_y = segment[2], segment[3]
        robot_x, robot_y = *self.chassis.position()
        difference_x = abs(end_point_x) - abs(robot_x)
        difference_y = abs(end_point_y) - abs(robot_y)
        if math.sqrt(difference_x ** 2 + difference_y ** 2) < self.ending_tolerance:
            self.current_waypoint_number += 1


    def sgn(self, number):
        """Returns the sign of a number, 0 is positive"""
        if number < 0:
            return -1
        else:
            return 1
