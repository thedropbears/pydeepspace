import math

import ctre
import numpy as np

from pyswervedrive.module import SwerveModule
from utilities.functions import constrain_angle


class PhysicsEngine:

    X_WHEELBASE = 0.50
    Y_WHEELBASE = 0.62
    GRAVITY = 9.8

    def __init__(self, controller):
        self.controller = controller

        self.drive_counts_per_rev = (
            SwerveModule.SRX_MAG_COUNTS_PER_REV
            * SwerveModule.DRIVE_ENCODER_GEAR_REDUCTION
        )
        self.drive_counts_per_meter = self.drive_counts_per_rev / (
            math.pi * SwerveModule.WHEEL_DIAMETER
        )

        # factor by which to scale velocities in m/s to give to our drive talon.
        # 0.1 is because SRX velocities are measured in ticks/100ms
        self.drive_velocity_to_native_units = self.drive_counts_per_meter * 0.1

        # for modules [a, b, c, d]. used to iterate over them
        self.module_steer_can_ids = [1, 3, 5, 7]
        self.module_drive_can_ids = [2, 4, 6, 8]
        self.module_steer_offsets = [0] * 4
        x_off = self.X_WHEELBASE / 2
        y_off = self.Y_WHEELBASE / 2
        self.module_x_offsets = [x_off, -x_off, -x_off, x_off]
        self.module_y_offsets = [y_off, y_off, -y_off, -y_off]

        self.controller.add_device_gyro_channel("navxmxp_spi_4_angle")

    def initialize(self, hal_data):
        pass

    def update_sim(self, hal_data, now, tm_diff):
        """Update pyfrc simulator.

        Args:
            hal_data: Data about motors and other components
            now: Current time in ms
            tm_diff: Difference between current time and time when last checked
        """
        # only simulate things if the robot is enabled
        if not hal_data["control"]["enabled"]:
            return

        steer_positions = []
        for can_id, offset in zip(self.module_steer_can_ids, self.module_steer_offsets):
            value = hal_data["CAN"][can_id]["pid0_target"]
            hal_data["CAN"][can_id]["pulse_width_position"] = int(value)
            position = constrain_angle(
                (hal_data["CAN"][can_id]["pulse_width_position"] - offset)
                / SwerveModule.STEER_COUNTS_PER_RADIAN
            )
            steer_positions.append(position)

        motor_speeds = []
        for i, can_id in enumerate(self.module_drive_can_ids):
            talon = hal_data["CAN"][can_id]
            if talon["control_mode"] == ctre.ControlMode.Velocity:
                enc_speed = talon["pid0_target"]
            else:
                enc_speed = 0
            speed = enc_speed / SwerveModule.drive_velocity_to_native_units
            talon["quad_position"] += int(enc_speed * tm_diff * 10)
            talon["quad_velocity"] = int(enc_speed)
            motor_speeds.append(speed)

        lf_speed, lr_speed, rr_speed, rf_speed = motor_speeds

        lf_angle, lr_angle, rr_angle, rf_angle = steer_positions
        vx, vy, vw = better_four_motor_swerve_drivetrain(
            motor_speeds, steer_positions, self.module_x_offsets, self.module_y_offsets
        )
        # convert meters to ft. (cause america)
        vx /= 0.3048
        vy /= 0.3048
        self.controller.vector_drive(-vy, vx, -vw, tm_diff)


def better_four_motor_swerve_drivetrain(
    module_speeds, module_angles, module_x_offsets, module_y_offsets
):
    """Solve the least-squares of the speed and angles of four swerve modules
    to retrieve delta x and y in the robot frame.

    Note:
        This function uses the standard (and superior) ROS coordinate system,
        with forward being positive x, leftward being positive y, and a
        counter clockwise rotation being one about the positive z axis.
    Args:
        module_speeds: List of the speeds of each module (m/s)
        module_angles: List of the angles of each module (radians)
        module_x_offsets: Offset of each module on the x axis.
        module_y_offsets: Offset of each module on the y axis.
    Returns:
        vx: float, robot velocity on the x axis (m/s)
        vy: float, robot velocity on the y axis (m/s)
        vz: float, robot velocity about the z axis (radians/s)
    """
    A = np.array(
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
    module_states = np.zeros((8, 1), dtype=float)
    for i in range(4):
        module_dist = math.hypot(module_x_offsets[i], module_y_offsets[i])
        module_angle = math.atan2(module_y_offsets[i], module_x_offsets[i])
        A[i * 2, 2] = -module_dist * math.sin(module_angle)
        A[i * 2 + 1, 2] = module_dist * math.cos(module_angle)

        x_vel = module_speeds[i] * math.cos(module_angles[i])
        y_vel = module_speeds[i] * math.sin(module_angles[i])
        module_states[i * 2, 0] = x_vel
        module_states[i * 2 + 1, 0] = y_vel

    lstsq_ret = np.linalg.lstsq(A, module_states, rcond=None)
    vx, vy, vz = lstsq_ret[0].reshape(3)

    return vx, vy, vz
