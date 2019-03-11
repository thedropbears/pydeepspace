import time

from collections import deque
from typing import Deque, NamedTuple, Tuple

import hal
from networktables import NetworkTablesInstance

from pyswervedrive.chassis import SwerveChassis
from utilities.functions import rotate_vector


class Odometry(NamedTuple):
    x: float
    y: float
    heading: float
    t: float


class Vision:

    chassis: SwerveChassis

    # NOTE: x and y are relative to the robot co-ordinate system, not the camera

    @property
    def fiducial_x(self) -> float:
        return self.fiducial_x_entry.getDouble(0.0)

    @property
    def fiducial_y(self) -> float:
        return self.fiducial_y_entry.getDouble(0.0)

    @property
    def fiducial_time(self) -> float:
        return self.fiducial_time_entry.getDouble(-1.0)

    @property
    def ping_time(self) -> float:
        return self.ping_time_entry.getDouble(0.0)

    @ping_time.setter
    def ping_time(self, value: float) -> None:
        self.ping_time_entry.setDouble(value)

    @property
    def raspi_pong_time(self) -> float:
        return self.raspi_pong_time_entry.getDouble(0.0)

    @property
    def rio_pong_time(self) -> float:
        return self.rio_pong_time_entry.getDouble(0.0)

    @property
    def latency(self) -> float:
        return self.latency_entry.getDouble(0.0)

    @latency.setter
    def latency(self, value: float) -> None:
        self.latency_entry.setDouble(value)

    @property
    def processing_time(self) -> float:
        return self.processing_time_entry.getDouble(0.0)

    @processing_time.setter
    def processing_time(self, value: float) -> None:
        self.processing_time_entry.setDouble(value)

    @property
    def camera(self) -> float:
        return self.camera_entry.getDouble(0)

    @camera.setter
    def camera(self, value: float) -> None:
        self.camera_entry.setDouble(value)

    def __init__(self) -> None:
        self.last_pong = time.monotonic()
        # 50Hz control loop for 2 seconds
        self.odometry: Deque[Odometry] = deque(maxlen=50 * 2)

        self.ntinst = NetworkTablesInstance()
        if hal.isSimulation():
            self.ntinst.startTestMode(server=False)
        else:
            self.ntinst.startClient("10.47.74.6")  # Raspberry pi's IP
        self.ntinst.setUpdateRate(1)  # ensure our flush calls flush immediately

        self.fiducial_x_entry = self.ntinst.getEntry("/vision/fiducial_x")
        self.fiducial_y_entry = self.ntinst.getEntry("/vision/fiducial_y")
        self.fiducial_time_entry = self.ntinst.getEntry("/vision/fiducial_time")
        self.ping_time_entry = self.ntinst.getEntry("/vision/ping")
        self.raspi_pong_time_entry = self.ntinst.getEntry("/vision/raspi_pong")
        self.rio_pong_time_entry = self.ntinst.getEntry("/vision/rio_pong")
        self.latency_entry = self.ntinst.getEntry("/vision/clock_offset")
        self.processing_time_entry = self.ntinst.getEntry("/vision/processing_time")
        self.camera_entry = self.ntinst.getEntry("/vision/game_piece")

    def execute(self) -> None:
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
        self.ntinst.flush()

    @property
    def fiducial_in_sight(self) -> bool:
        return time.monotonic() - (self.fiducial_time + self.latency) < 0.1

    def get_fiducial_position(self) -> Tuple[float, float, float]:
        """Return the position of the retroreflective fiducials relative to the current robot pose."""
        vision_time = self.fiducial_time + self.latency
        vision_delta_x, vision_delta_y, vision_delta_heading = self._get_pose_delta(
            vision_time
        )
        x = self.fiducial_x - vision_delta_x
        y = self.fiducial_y - vision_delta_y
        return x, y, vision_delta_heading

    def _get_pose_delta(self, t: float) -> Tuple[float, float, float]:
        """Search the stored odometry and return the position difference between now and the specified time."""
        current = self.odometry[0]
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

    def ping(self) -> None:
        """Send a ping to the RasPi to determine the connection latency."""
        self.ping_time = time.monotonic()

    def pong(self) -> None:
        """Receive a pong from the RasPi to determine the connection latency."""
        if abs(self.rio_pong_time - self.last_pong) > 1e-4:  # Floating point comparison
            alpha = 0.0  # Exponential averaging
            self.latency = alpha * self.latency + (1 - alpha) * (
                self.rio_pong_time - self.raspi_pong_time
            )
            self.last_pong = self.rio_pong_time
