from components.vision import Odometry, Vision
from utilities.functions import rotate_vector

import math
import time


class FakeImu:
    def __init__(self):
        self.angle = 0.0

    def getAngle(self):
        return self.angle


class FakeChassis:
    def __init__(self):
        self.odometry_x = 0.0
        self.odometry_y = 0.0
        self.imu = FakeImu()


def init_vision(heading=0.0):
    t = time.monotonic()
    v = Vision()
    v.chassis = FakeChassis()
    # Inject some fake odometry
    v.odometry.appendleft(Odometry(0, 0, heading, t - 0.3))
    v.odometry.appendleft(Odometry(1, 1, heading, t - 0.2))
    v.odometry.appendleft(Odometry(2, 2, heading, t - 0.1))
    v.odometry.appendleft(Odometry(3, 3, heading, t))
    return v, t


def test_odom_deque_order():
    v = Vision()
    v.chassis = FakeChassis()
    for i in range(5):
        time.sleep(0.05)
        v.execute()
    assert len(v.odometry) > 0
    newer_odom = v.odometry.popleft()
    for odom in v.odometry:
        assert newer_odom.t > odom.t
        newer_odom = odom


def test_get_pose_delta():
    v, t = init_vision()

    x, y, heading = v._get_pose_delta(t)
    assert x == 0.0
    assert y == 0.0

    x, y, heading = v._get_pose_delta(t - 0.1)
    assert x == 1.0
    assert y == 1.0

    x, y, heading = v._get_pose_delta(t - 0.2)
    assert x == 2.0
    assert y == 2.0


def test_get_fiducial_position():
    v, t = init_vision()

    v.fiducial_x = 5.0
    v.fiducial_y = 0.5

    v.fiducial_time = t
    x, y, heading = v.get_fiducial_position()
    assert x == 5.0
    assert y == 0.5
    assert heading == 0.0

    v.fiducial_time = t - 0.15
    x, y, heading = v.get_fiducial_position()
    assert x == 4.0
    assert y == -0.5
    assert heading == 0.0

    v.fiducial_time = t - 0.25
    x, y, heading = v.get_fiducial_position()
    assert x == 3.0
    assert y == -1.5
    assert heading == 0.0


def test_get_fiducial_position_rotated():
    v, t = init_vision(math.pi / 2)
    v.chassis.imu.heading = math.pi / 2

    v.fiducial_x = 5.0
    v.fiducial_y = 0.5

    v.fiducial_time = t
    x, y, heading = v.get_fiducial_position()
    assert x == 5.0
    assert y == 0.5
    assert heading == 0.0

    v.fiducial_time = t - 0.15
    x, y, heading = v.get_fiducial_position()
    assert x == 4.0
    assert y == 1.5
    assert heading == 0.0

    v.fiducial_time = t - 0.25
    x, y, heading = v.get_fiducial_position()
    assert x == 3.0
    assert y == 2.5
    assert heading == 0.0


def test_odometry_rotation():
    v, t = init_vision()
    v.chassis.imu.heading = math.pi / 2
    odom = v.odometry.popleft()
    v.odometry.appendleft(Odometry(odom.x, odom.y, math.pi / 2, odom.t))

    v.fiducial_x = 5.0
    v.fiducial_y = 0.5

    v.fiducial_time = t
    x, y, heading = v.get_fiducial_position()
    assert x == 5.0
    assert y == 0.5
    assert heading == 0.0

    v.fiducial_time = t - 0.15
    x, y, heading = v.get_fiducial_position()
    assert x == 4.0
    assert y == -0.5
    assert heading == math.pi / 2

    rot_x, rot_y = rotate_vector(x, y, -heading)
    assert abs(rot_x - -0.5) < 1e-4
    assert abs(rot_y - -4) < 1e-4
