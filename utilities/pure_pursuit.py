import math
from typing import List, Optional, Sequence, Tuple

import numpy as np

#: A point in 2D cartesian space.
Cartesian2D = Tuple[float, float]
#: x, y, theta, speed
Waypoint = Tuple[float, float, float, float]
#: A Waypoint with an additional cumulative distance.
Segment = Tuple[float, float, float, float, float]


class PurePursuit:
    """
    Pure Pursuit controller for navigation with absolute waypoints.

    Uses the method outlined here with some changes to be suitible for a swervedrive
    https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf
    """

    waypoints: List[Segment]

    def __init__(self, look_ahead: float, look_ahead_speed_modifier: float):
        self.waypoints = []
        self.current_waypoint_number = 0
        self.look_ahead = look_ahead
        self.look_ahead_speed_modifier = look_ahead_speed_modifier
        self.speed_look_ahead = look_ahead
        self.completed_path = False
        self.distance_traveled = 0.0

    def find_intersections(
        self,
        waypoint_start: Segment,
        waypoint_end: Segment,
        robot_position: Cartesian2D,
    ) -> Optional[np.ndarray]:
        """
        Find the intersection/s between our lookahead distance and path.

        http://mathworld.wolfram.com/Circle-LineIntersection.html
        NOTE: this will return the intersections in global co-ordinates
        """
        x_1, y_1 = waypoint_start[0], waypoint_start[1]
        x_2, y_2 = waypoint_end[0], waypoint_end[1]
        robot_x, robot_y = robot_position
        x_2 -= robot_x
        x_1 -= robot_x
        y_2 -= robot_y
        y_1 -= robot_y
        segment_end = np.array((x_2, y_2))

        d_x = x_2 - x_1
        d_y = y_2 - y_1
        d_r = math.sqrt(d_x ** 2 + d_y ** 2)
        D = x_1 * y_2 - x_2 * y_1
        r = self.speed_look_ahead
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
            if np.linalg.norm((intersection_1) - (segment_end)) < np.linalg.norm(
                (intersection_2) - (segment_end)
            ):
                return intersection_1
            else:
                return intersection_2
        else:
            print("No intersection found")

    def build_path(self, waypoints: Sequence[Waypoint]) -> None:
        """
        Take in a list of waypoints used to build a path.

        The waypoints must be a tuple (x, y, theta, speed), this method will
        create waypoints with these co-ordinates and distance
        along the path from the start of the trajectory.
        """
        self.last_robot_x = waypoints[0][0]
        self.last_robot_y = waypoints[0][1]
        self.completed_path = False
        self.distance_traveled = 0
        self.waypoints = []
        waypoint_distance = 0.0
        previous_waypoint = waypoints[0]
        for waypoint in waypoints:
            x, y, theta, speed = waypoint
            # print(waypoint)
            waypoint_distance += math.hypot(
                x - previous_waypoint[0], y - previous_waypoint[1]
            )
            previous_waypoint = waypoint
            self.waypoints.append((x, y, theta, speed, waypoint_distance))
        self.current_waypoint_number = 0
        print(f"waypoints = {self.waypoints}")

    def compute_direction(
        self,
        robot_position: Cartesian2D,
        segment_start: Segment,
        segment_end: Segment,
        distance_along_path: float,
    ) -> np.ndarray:
        """Find the goal_point and convert it to relative co-ordinates"""
        goal_point = self.find_intersections(segment_start, segment_end, robot_position)
        if goal_point is None:
            # if we cant find an intersection between the look_ahead and path
            # use the next waypoint as our goal point
            goal_point = segment_end[:2]
        # print(goal_point)
        goal_point = goal_point / np.linalg.norm(goal_point)
        return goal_point

    @staticmethod
    def sgn(number: float) -> int:
        """Returns the sign of a number, 0 is positive"""
        if number < 0:
            return -1
        else:
            return 1

    def distance_along_path(self, robot_position: Cartesian2D) -> float:
        """
        Find the robots position on the path using odometry.
        Every timestep, add the distance the robot has travelled to a
        running total used to check for waypoints.
        """
        robot_x, robot_y = robot_position
        self.distance_traveled += math.hypot(
            robot_x - self.last_robot_x, robot_y - self.last_robot_y
        )
        self.last_robot_x = robot_x
        self.last_robot_y = robot_y
        # print(self.distance_traveled)
        return self.distance_traveled

    def find_speed(
        self,
        start_path_distance: float,
        end_path_distance: float,
        start_speed: float,
        end_speed: float,
        distance_along_path: float,
    ) -> float:
        """
        Find the how fast the robot should be moving at its current point.
        """
        local_robot_distance = distance_along_path - start_path_distance
        local_end_distance = end_path_distance - start_path_distance
        speed_difference = end_speed - start_speed
        portion_path_completed = local_robot_distance / local_end_distance
        target_speed = speed_difference * portion_path_completed + start_speed
        return target_speed

    def find_velocity(self, robot_position: Cartesian2D) -> Tuple[float, float, float]:
        if self.current_waypoint_number >= len(self.waypoints) - 1:
            self.completed_path = True
            print("WARNING: path completed")
            return 0, 0, 0
        distance_along_path = self.distance_along_path(robot_position)
        segment_start = self.waypoints[self.current_waypoint_number]
        segment_end = self.waypoints[self.current_waypoint_number + 1]
        start_speed, start_distance = segment_start[3:]
        end_speed, end_distance = segment_end[3:]
        direction = self.compute_direction(
            robot_position, segment_start, segment_end, distance_along_path
        )
        speed = self.find_speed(
            start_distance, end_distance, start_speed, end_speed, distance_along_path
        )
        vx, vy = direction * speed
        heading = segment_end[2]
        self.speed_look_ahead = self.look_ahead + self.look_ahead_speed_modifier * speed
        if self.distance_traveled + self.speed_look_ahead >= segment_end[4]:
            # if we have reached the end of our current segment
            self.current_waypoint_number += 1
            print("changed segment")
        return vx, vy, heading


def insert_trapezoidal_waypoints(
    waypoints: Sequence[Waypoint], acceleration: float, deceleration: float
) -> List[Waypoint]:
    """Generate how far you have to travel to accelerate and decelerate for speed control.

    Assumes that the robot should accelerate then cruise when v_init < v_final,
    otherwise we cruise then decelerate.

    Args:
        acceleration: acceleration when increasing speed
        deceleration: acceleration when decreasing speed
    """
    trap_waypoints = []
    for idx in range(len(waypoints) - 1):
        segment_start = waypoints[idx]
        segment_end = waypoints[idx + 1]

        dx = segment_end[0] - segment_start[0]
        dy = segment_end[1] - segment_start[1]

        segment_distance = math.hypot(dx, dy)
        u = segment_start[3]
        v = segment_end[3]

        trap_waypoints.append(segment_start)
        if v > u:
            # Faster at the end - accelerating
            a = acceleration
            # Rearrange v^2 = u^2 + 2as
            s = (v ** 2 - u ** 2) / (2 * a)
            if s > segment_distance:
                # Cannot actually get to speed in time
                # Leave the segments as they are
                continue
            intermediate = (
                dx * s / segment_distance + segment_start[0],
                dy * s / segment_distance + segment_start[1],
            ) + segment_end[2:]
            trap_waypoints.append(intermediate)

        elif u > v:
            a = deceleration
            # Rearrange v^2 = u^2 + 2as, then subtract from the segment length
            s = segment_distance - (v ** 2 - u ** 2) / (2 * a)
            if s < 0:
                # Not enough time to decelerate
                # Leave the segments as they are
                continue
            intermediate = (
                dx * s / segment_distance + segment_start[0],
                dy * s / segment_distance + segment_start[1],
            ) + segment_start[2:]
            trap_waypoints.append(intermediate)

    trap_waypoints.append(waypoints[-1])
    return trap_waypoints
