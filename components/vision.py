import math
import time

from collections import deque

from networktables import NetworkTables
from networktables.util import ntproperty

from pyswervedrive.chassis import SwerveChassis


class Vision:

    chassis: SwerveChassis

    fiducial_x = ntproperty("/vision/fiducial_x", 0.0, writeDefault=False)
    fiducial_y = ntproperty("/vision/fiducial_y", 0.0, writeDefault=False)
    fiducial_time = ntproperty("/vision/fiducial_time", -1.0, writeDefault=False)
    ping_time = ntproperty("/vision/ping", 0.0, writeDefault=False)
    raspi_pong_time = ntproperty("/vision/raspi_pong", 0.0, writeDefault=False)
    rio_pong_time = ntproperty("/vision/rio_pong", 0.0, writeDefault=False)
    latency = ntproperty("/vision/clock_offset", 0.0)
    # NOTE: x and y are relative to the robot co-ordinate system, not the camera

    def __init__(self):
        self.latency = 0.0
        self.last_pong = time.monotonic()
        self.odometry = deque(maxlen=50 * 2)  # 50Hz control loop for 2 secs

    def execute(self):
        """Store the current odometry in the queue. Allows projection of target into current position."""
        self.odometry.appendleft(
            (
                self.chassis.odometry_x,
                self.chassis.odometry_y,
                self.chassis.imu.getAngle(),
                time.monotonic(),
            )
        )
        self.ping()
        self.pong()
        NetworkTables.flush()

    @property
    def fiducial_in_sight(self):
        return time.monotonic() - (self.fiducial_time + self.latency) < 0.5

    def get_fiducial_position(self):
        """Return the position of the retroreflective fiducials relative to the current robot pose."""
        vision_time = self.fiducial_time + self.latency
        if self.fiducial_in_sight:  # Only return if not too stale
            vision_delta_x, vision_delta_y, vision_delta_heading = self._get_pose_delta(
                vision_time
            )
            x = self.fiducial_x - vision_delta_x
            y = self.fiducial_y - vision_delta_y
            return x, y, vision_delta_heading
        return None, None, None

    def _get_pose_delta(self, t):
        """Search the stored odometry and return the position difference between now and the specified time."""
        current = self.odometry[0]
        previous = self.odometry[0]
        for odom in self.odometry:
            if odom[3] > t:
                previous = odom
            else:
                break

        x = current[0] - previous[0]
        y = current[1] - previous[1]
        # Rotate to the robot frame of reference
        # Use the previous heading - that's where we were when the picture was taken
        heading = previous[2]
        robot_x = x * math.cos(heading) + y * math.sin(heading)
        robot_y = -y * math.sin(heading) + y * math.cos(heading)
        return robot_x, robot_y, current[2] - heading

    def ping(self):
        """Send a ping to the RasPi to determine the connection latency."""
        self.ping_time = time.monotonic()

    def pong(self):
        """Receive a pong from the RasPi to determine the connection latency."""
        if abs(self.rio_pong_time - self.last_pong) > 1e-4:  # Floating point comparison
            alpha = 0.9  # Exponential averaging
            self.latency = alpha * self.latency + (1 - alpha) * (
                self.rio_pong_time - self.raspi_pong_time
            )
            self.last_pong = self.rio_pong_time
