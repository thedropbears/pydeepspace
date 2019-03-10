import math
import time
from typing import Tuple

import numpy as np
from magicbot import tunable
from wpilib_controller import PIDController

from utilities.navx import NavX
from .module import SwerveModule


class SwerveChassis:
    WIDTH = 0.75
    LENGTH = 0.75

    imu: NavX
    module_a: SwerveModule
    module_b: SwerveModule
    module_c: SwerveModule
    module_d: SwerveModule

    # tunables here purely for debugging
    odometry_x = tunable(0)
    odometry_y = tunable(0)
    # odometry_x_vel = tunable(0)
    # odometry_y_vel = tunable(0)
    # odometry_z_vel = tunable(0)

    hold_heading = tunable(True)

    def __init__(self):
        self.vx = 0
        self.vy = 0
        self.vz = 0
        self.field_oriented = False
        self.momentum = False
        self.automation_running = False

    def setup(self):
        # Heading PID controller
        self.heading_pid = PIDController(
            Kp=6.0, Ki=0.0, Kd=0.05, measurement_source=self.imu.getAngle, period=1 / 50
        )
        self.heading_pid.setInputRange(-math.pi, math.pi)
        self.heading_pid.setOutputRange(-3, 3)
        self.heading_pid.setContinuous()
        self.modules = [self.module_a, self.module_b, self.module_c, self.module_d]

        self.odometry_x = 0
        self.odometry_y = 0
        self.odometry_x_vel = 0
        self.odometry_y_vel = 0
        self.odometry_z_vel = 0

        self.A = np.array(
            [
                [1, 0, 1],
                [0, 1, 1],
                [1, 0, 1],
                [0, 1, 1],
                [1, 0, 1],
                [0, 1, 1],
                [1, 0, 1],
                [0, 1, 1],
            ],
            dtype=float,
        )

        self.last_odometry_time = 0
        # wpilib.SmartDashboard.putData("heading_pid", self.heading_pid)

        # figure out the contribution of the robot's overall rotation about the
        # z axis to each module's movement, and encode that information in a

        for i, module in enumerate(self.modules):
            module_dist = math.hypot(module.x_pos, module.y_pos)
            module_angle = math.atan2(module.y_pos, module.x_pos)
            # self.z_axis_adjustment[i*2, 0] = -module_dist*math.sin(module_angle)
            # self.z_axis_adjustment[i*2+1, 0] = module_dist*math.cos(module_angle)
            self.A[i * 2, 2] = -module_dist * math.sin(module_angle)
            self.A[i * 2 + 1, 2] = module_dist * math.cos(module_angle)

    def set_heading_sp_current(self):
        self.set_heading_sp(self.imu.getAngle())

    def set_heading_sp(self, setpoint):
        self.heading_pid.setReference(setpoint)

    def heading_hold_on(self):
        self.set_heading_sp_current()
        self.heading_pid.reset()
        self.hold_heading = True

    def heading_hold_off(self):
        self.hold_heading = False

    def on_enable(self):
        self.heading_hold_on()

        self.last_heading = self.imu.getAngle()
        self.odometry_updated = False
        for module in self.modules:
            module.reset_encoder_delta()

        self.last_odometry_time = time.monotonic()

    def execute(self):

        pid_z = 0
        if self.hold_heading:
            pid_z = self.heading_pid.update()
            if self.momentum and abs(self.imu.getHeadingRate()) < 0.005:
                self.momentum = False

            if self.vz not in [0.0, None]:
                self.momentum = True

            if self.momentum:
                self.set_heading_sp_current()
                pid_z = 0

        input_vz = 0
        if self.vz is not None:
            input_vz = self.vz
        if abs(pid_z) < 0.1 and math.hypot(self.vx, self.vy) < 0.01:
            pid_z = 0
        vz = input_vz + pid_z

        angle = self.imu.getAngle()

        # TODO: re-enable if we end up not using callback method
        self.update_odometry()
        # self.odometry_updated = False  # reset for next timestep

        for module in self.modules:
            # Calculate the additional vx and vy components for this module
            # required to achieve our desired angular velocity
            vz_x = -module.dist * vz * math.sin(module.angle)
            vz_y = module.dist * vz * math.cos(module.angle)
            if self.field_oriented:
                vx, vy = self.robot_orient(self.vx, self.vy, angle)
            else:
                vx, vy = self.vx, self.vy
            module.set_velocity(vx + vz_x, vy + vz_y, absolute_rotation=False)

    def update_odometry(self, *args):
        # TODO: re-enable if we end up not using callback method
        # if self.odometry_updated:
        #     return
        heading = self.imu.getAngle()

        odometry_outputs = np.zeros((8, 1))
        velocity_outputs = np.zeros((8, 1))

        # betas = []
        # phi_dots = []
        for i, module in enumerate(self.modules):
            module.update_odometry()
            odometry_x, odometry_y = module.get_cartesian_delta()
            velocity_x, velocity_y = module.get_cartesian_vel()
            odometry_outputs[i * 2, 0] = odometry_x
            odometry_outputs[i * 2 + 1, 0] = odometry_y
            velocity_outputs[i * 2, 0] = velocity_x
            velocity_outputs[i * 2 + 1, 0] = velocity_y
            module.reset_encoder_delta()
            # betas.append(module.measured_azimuth)
            # phi_dots.append(module.wheel_angular_vel)

        # q = np.array(betas)
        # lambda_e = self.icre.estimate_lmda(q)
        # print(lambda_e)

        now = time.monotonic()
        vx, vy, vz = self.robot_movement_from_odometry(velocity_outputs, heading)
        # delta_x, delta_y, delta_z = self.robot_movement_from_odometry(
        # odometry_outputs, heading, z_vel=self.imu.getHeadingRate()
        # )

        delta_t = now - self.last_odometry_time
        delta_x = vx * delta_t
        delta_y = vy * delta_t

        self.odometry_x += delta_x
        self.odometry_y += delta_y
        self.odometry_x_vel = vx
        self.odometry_y_vel = vy
        self.odometry_z_vel = vz

        self.last_heading = heading

        self.odometry_updated = True

        self.set_modules_drive_brake()

        self.last_odometry_time = now

    def robot_movement_from_odometry(self, odometry_outputs, angle, z_vel=0):
        lstsq_ret = np.linalg.lstsq(self.A, odometry_outputs, rcond=None)
        x, y, theta = lstsq_ret[0].reshape(3)
        # TODO: re-enable if we move back to running in the same thread
        x_field, y_field = self.field_orient(x, y, angle + z_vel * (1 / 200))
        # x_field, y_field = self.field_orient(x, y, angle)
        return x_field, y_field, theta

    def set_velocity_heading(self, vx, vy, heading):
        """Set a translational velocity and a rotational orientation to achieve.

        Args:
            vx: (forward) component of the robot's desired velocity. In m/s.
            vy: (leftward) component of the robot's desired velocity. In m/s.
            heading: the heading the robot is to face.
        """
        self.vx = vx
        self.vy = vy
        self.vz = None
        self.set_heading_sp(heading)

    def set_inputs(
        self, vx: float, vy: float, vz: float, *, field_oriented: bool = True
    ):
        """Set chassis vx, vy, and vz components of inputs.
        Args:
            vx: (forward) component of the robot's desired velocity. In m/s.
            vy: (leftward) component of the robot's desired velocity. In m/s.
            vz: The vz (counter-clockwise rotation) component of the robot's
                desired [angular] velocity. In radians/s.
            field_oriented: Whether the inputs are field or robot oriented.
        """
        self.vx = vx
        self.vy = vy
        self.vz = vz
        self.field_oriented = field_oriented

    @staticmethod
    def robot_orient(vx: float, vy: float, heading: float) -> Tuple[float, float]:
        """Turn a vx and vy relative to the field into a vx and vy based on the
        robot.

        Args:
            vx: vx to robot orient
            vy: vy to robot orient
            heading: current heading of the robot in radians CCW from +x axis.

        Returns:
            tuple of robot oriented vx and vy
        """
        c_h = math.cos(heading)
        s_h = math.sin(heading)
        oriented_vx = vx * c_h + vy * s_h
        oriented_vy = -vx * s_h + vy * c_h
        return oriented_vx, oriented_vy

    @staticmethod
    def field_orient(vx: float, vy: float, heading: float) -> Tuple[float, float]:
        """Turn a vx and vy relative to the robot into a vx and vy based on the
        field.

        Args:
            vx: vx to field orient
            vy: vy to field orient
            heading: current heading of the robot in radians CCW from +x axis.

        Returns:
            tuple of field oriented vx and vy
        """
        c_h = math.cos(heading)
        s_h = math.sin(heading)
        oriented_vx = vx * c_h - vy * s_h
        oriented_vy = vx * s_h + vy * c_h
        return oriented_vx, oriented_vy

    @property
    def position(self):
        return self.odometry_x, self.odometry_y

    @property
    def speed(self):
        return math.hypot(self.odometry_x_vel, self.odometry_y_vel)

    @property
    def all_aligned(self):
        return all(module.aligned for module in self.modules)

    def set_modules_drive_coast(self):
        for module in self.modules:
            module.set_drive_coast()

    def set_modules_drive_brake(self):
        for module in self.modules:
            module.set_drive_brake()
