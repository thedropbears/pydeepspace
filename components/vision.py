import time

from collections import deque

from networktables import NetworkTables
from networktables.util import ntproperty

from pyswervedrive.chassis import SwerveChassis

from typing import NamedTuple

from utilities.functions import rotate_vector


class Odometry(NamedTuple):
    x: float
    y: float
    heading: float
    t: float


class Vision:

    chassis: SwerveChassis

    fiducial_x = ntproperty("/vision/fiducial_x", 0.0, writeDefault=False)
    fiducial_y = ntproperty("/vision/fiducial_y", 0.0, writeDefault=False)
    fiducial_time = ntproperty("/vision/fiducial_time", -1.0, writeDefault=False)
    ping_time = ntproperty("/vision/ping", 0.0, writeDefault=False)
    raspi_pong_time = ntproperty("/vision/raspi_pong", 0.0, writeDefault=False)
    rio_pong_time = ntproperty("/vision/rio_pong", 0.0, writeDefault=False)
    latency = ntproperty("/vision/clock_offset", 0.0)
    processing_time = ntproperty("/vision/processing_time", 0.0)
    # NOTE: x and y are relative to the robot co-ordinate system, not the camera

    def __init__(self):
        self.latency = 0.0
        self.last_pong = time.monotonic()
        self.odometry = deque(maxlen=50 * 2)  # 50Hz control loop for 2 secs
        NetworkTables.setUpdateRate(1)  # ensure our flush calls flush immediately

    def execute(self):
        """Store the current odometry in the queue. Allows projection of target into current position."""
        self.odometry.appendleft(
            Odometry(
                self.chassis.odometry_x,
                self.chassis.odometry_y,
                self.chassis.imu.getAngle(),
                time.monotonic(),
            )
        )
        self.ping()
        self.pong()
        vision_time = self.fiducial_time + self.latency
        self.processing_time = time.monotonic() - vision_time
        NetworkTables.flush()

    @property
    def fiducial_in_sight(self):
        return time.monotonic() - (self.fiducial_time + self.latency) < 0.5

    def get_fiducial_position(self):
        """Return the position of the retroreflective fiducials relative to the current robot pose."""
        vision_time = self.fiducial_time + self.latency
        vision_delta_x, vision_delta_y, vision_delta_heading = self._get_pose_delta(
            vision_time
        )
        x = self.fiducial_x - vision_delta_x
        y = self.fiducial_y - vision_delta_y
        return x, y, vision_delta_heading

    def _get_pose_delta(self, t):
        """Search the stored odometry and return the position difference between now and the specified time."""
        current = self.odometry[0]
        previous = self.odometry[0]
        for odom in self.odometry:
            if odom.t >= t:
                previous = odom
            else:
                break

        x = current.x - previous.x
        y = current.y - previous.y
        # Rotate to the robot frame of reference
        # Use the previous heading - that's where we were when the picture was taken
        heading = previous.heading
        robot_x, robot_y = rotate_vector(x, y, -heading)
        return robot_x, robot_y, current.heading - heading

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
