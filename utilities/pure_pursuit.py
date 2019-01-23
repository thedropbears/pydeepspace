import math
import numpy as np
# from magicbot import tunable


class PurePursuit:
    """
    Pure Pursuit controller for navigation with absolute waypoints

    Uses the method outlined here
    https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf
    """

    # look_ahead = tunable(0) # m
    # speed_modifier = tunable(0)
    # ending_tolerance = tunable(0) # m

    def __init__(self, look_ahead, speed_modifier, ending_tolerance):
        self.waypoints = []
        self.current_waypoint_number = 0
        self.look_ahead = look_ahead
        self.speed_modifier = speed_modifier
        self.ending_tolerance = ending_tolerance

    def find_intersections(self, waypoint_start, waypoint_end, robot_position):
        """
        Find the intersection/s between our lookahead distance and path.

        http://mathworld.wolfram.com/Circle-LineIntersection.html
        NOTE: this will return the intersections in global co-ordinates
        """
        x_1, y_1 = waypoint_start[0], waypoint_start[1]
        x_2, y_2 = waypoint_end[0], waypoint_end[1]
        robot_x, robot_y, heading = robot_position
        x_2 -= robot_x
        x_1 -= robot_x
        y_2 -= robot_y
        y_1 -= robot_y

        d_x = x_2 - x_1
        d_y = y_2 - y_1
        d_r = math.sqrt(d_x ** 2 + d_y ** 2)
        # D = np.array(((x_1, y_1), (x_2, y_2))) will need to be vectorised
        D = x_1 * y_2 - x_2 * y_1
        r = self.look_ahead
        discriminent = r ** 2 * d_r ** 2 - D ** 2

        if discriminent >= 0:  # if an intersection exists
            intersection_1 = np.zeros((2))
            intersection_2 = np.zeros((2))
            sqrt_discriminent = math.sqrt(discriminent)

            right_x = self.sgn(d_y) * d_x * sqrt_discriminent
            left_x = D * d_y
            right_y = abs(d_y) * sqrt_discriminent
            left_y = -1 * D * d_x
            denominator = d_r ** 2
            if denominator == 0:
                print("Pursuit: caught division by zero")
                return
            intersection_1[0] = (left_x + right_x) / denominator
            intersection_1[1] = (left_y + right_y) / denominator
            if discriminent == 0:  # if we are tangent to our path
                return intersection_1
            intersection_2[0] = (left_x - right_x) / denominator
            intersection_2[1] = (left_y - right_y) / denominator
            if np.linalg.norm(abs(intersection_1) - abs(waypoint_end)) < np.linalg.norm(
                abs(intersection_2) - abs(waypoint_end)
            ):
                return intersection_1
            else:
                return intersection_2
        else:
            print("No intersection found")

    def build_path(self, waypoints: np.ndarray):
        self.waypoints = waypoints

    def compute_direction(self, robot_position):
        """Find the goal_point and convert it to relative co-ordinates"""
        if self.current_waypoint_number + 1 >= len(self.waypoints):
            return
        goal_point = self.find_intersections(
            self.waypoints[self.current_waypoint_number],
            self.waypoints[self.current_waypoint_number + 1],
            robot_position
        )
        goal_point -= robot_position[:2]
        goal_point = self.robot_orient(*goal_point, robot_position[2])
        self.check_progress(self.waypoints[self.current_waypoint_number + 1], robot_position)
        print(goal_point)
        return goal_point

    def check_progress(self, end_waypoint, robot_position):
        """Check if we are close enough to begin the path to the next end_waypoint"""
        end_point_x, end_point_y = end_waypoint[0], end_waypoint[1]
        robot_x, robot_y, heading = robot_position
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

    def robot_orient(self, x, y, heading):
        """Turn a vx and vy relative to the field into a vx and vy based on the
        robot.

        Args:
            vx: vx to robot orient
            vy: vy to robot orient
            heading: current heading of the robot. In radians CCW from +x axis.
        Returns:
            float: robot oriented vx speed
            float: robot oriented vy speed
        """
        oriented_x = x * math.cos(heading) + y * math.sin(heading)
        oriented_y = -x * math.sin(heading) + y * math.cos(heading)
        return oriented_x, oriented_y
