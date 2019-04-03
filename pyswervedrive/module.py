import math

import ctre
import hal
from networktables import NetworkTables

from utilities.functions import constrain_angle


class SwerveModule:

    DRIVE_FREE_SPEED: float = 94000.0  # encoder ticks / 100 ms
    # TODO: change back for real robot
    SRX_MAG_COUNTS_PER_REV: int = 4096
    WHEEL_DIAMETER: float = 0.0254 * 3
    DRIVE_ENCODER_GEAR_REDUCTION: float = 100 / 12 * 46 / 26
    STEER_COUNTS_PER_REV = 4096
    STEER_GEAR_REDUCTION: float = 49
    STEER_FREE_SPEED: float = 18000 / 60 / STEER_GEAR_REDUCTION * STEER_COUNTS_PER_REV / 10
    STEER_COUNTS_PER_RADIAN = STEER_COUNTS_PER_REV / math.tau

    drive_counts_per_rev = SRX_MAG_COUNTS_PER_REV * DRIVE_ENCODER_GEAR_REDUCTION
    drive_counts_per_radian = drive_counts_per_rev / math.tau
    # odometry is consistently slightly off, need a fudge factor to compensate
    # TODO: Tune the fudge factor
    drive_odometry_fudge_factor = 1 / 1
    drive_counts_per_metre = (
        drive_counts_per_rev / (math.pi * WHEEL_DIAMETER) * drive_odometry_fudge_factor
    )

    # factor by which to scale velocities in m/s to give to our drive talon.
    # 0.1 is because SRX velocities are measured in ticks/100ms
    drive_velocity_to_native_units = drive_counts_per_metre * 0.1
    drive_angular_vel_to_native_units = drive_counts_per_radian * 0.1

    def __init__(
        self,
        name: str,
        steer_talon: ctre.TalonSRX,
        drive_talon: ctre.TalonSRX,
        x_pos: float,
        y_pos: float,
        reverse_steer_direction: bool = False,
        reverse_steer_encoder: bool = True,
        reverse_drive_direction: bool = False,
        reverse_drive_encoder: bool = False,
    ):
        if hal.isSimulation():
            # we aren't using the PID simulation here
            steer_talon._use_notifier = False
            drive_talon._use_notifier = False

        self.name = name

        self.steer_motor = steer_talon
        self.drive_motor = drive_talon
        self.x_pos = x_pos
        self.y_pos = y_pos
        self.dist = math.hypot(self.x_pos, self.y_pos)
        self.angle = math.atan2(self.y_pos, self.x_pos)
        self.reverse_steer_direction = reverse_steer_direction
        self.reverse_steer_encoder = reverse_steer_encoder
        self.reverse_drive_direction = reverse_drive_direction
        self.reverse_drive_encoder = reverse_drive_encoder

        self.steer_enc_offset = self.steer_motor.configGetCustomParam(0, timoutMs=10)

        self.nt = NetworkTables.getTable("SwerveDrive").getSubTable(name)
        self.steer_enc_offset_entry = self.nt.getEntry("steer_enc_offset")
        self.steer_enc_offset_entry.setDouble(self.steer_enc_offset)
        self.steer_enc_offset_entry.addListener(
            self.nt_offset_changed, NetworkTables.NotifyFlags.UPDATE
        )

        self.last_speed = 0.0

        self.update_odometry()

        # NOTE: In all the following config* calls to the drive and steer
        # motors, the last argument is the timeout in milliseconds. See
        # robotpy-ctre documentation for details.

        self.steer_motor.configSelectedFeedbackSensor(
            ctre.FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10
        )
        # changes direction of motor encoder
        self.steer_motor.setSensorPhase(self.reverse_steer_encoder)
        # changes sign of motor throttle vilues
        self.steer_motor.setInverted(self.reverse_steer_direction)
        # self.steer_motor.setSelectedSensorPosition(0)

        self.steer_motor.selectProfileSlot(0, 0)
        self.steer_motor.config_kF(0, 1023 / self.STEER_FREE_SPEED, 10)
        self.steer_motor.config_kP(0, 0.75, 10)
        self.steer_motor.config_kI(0, 0.0, 10)
        self.steer_motor.config_kD(0, 0.0, 10)
        self.steer_motor.configMotionCruiseVelocity(2000, 10)
        self.steer_motor.configMotionAcceleration(10000, 10)

        self.steer_motor.configPeakCurrentLimit(10, timeoutMs=10)
        self.steer_motor.configPeakCurrentDuration(milliseconds=1, timeoutMs=10)
        self.steer_motor.configContinuousCurrentLimit(10, timeoutMs=10)
        # TODO tune all of this
        self.steer_motor.enableCurrentLimit(True)

        self.steer_motor.setNeutralMode(ctre.NeutralMode.Coast)

        self.drive_motor.configSelectedFeedbackSensor(
            ctre.FeedbackDevice.QuadEncoder, 0, 10
        )
        # changes direction of motor encoder
        self.drive_motor.setSensorPhase(self.reverse_drive_encoder)
        # changes sign of motor throttle values
        self.drive_motor.setInverted(self.reverse_drive_direction)
        # Reset drive encoder to 0
        self.drive_motor.setSelectedSensorPosition(0)
        # TODO: change back to original constants once we get on to real robot
        self.drive_motor.config_kP(0, 0.05, 10)
        self.drive_motor.config_kI(0, 0, 10)
        self.drive_motor.config_kD(0, 0, 10)
        self.drive_motor.config_kF(0, 1024.0 / self.DRIVE_FREE_SPEED, 10)
        self.drive_motor.configClosedLoopRamp(0.45, 10)
        self.drive_motor.selectProfileSlot(0, 0)

        self.drive_motor.setNeutralMode(ctre.NeutralMode.Brake)

        self.reset_encoder_delta()

        self.drive_motor.configVoltageCompSaturation(9, timeoutMs=10)
        self.drive_motor.configPeakCurrentLimit(50, timeoutMs=10)
        self.drive_motor.configContinuousCurrentLimit(40, timeoutMs=10)
        self.drive_motor.configPeakCurrentDuration(10, timeoutMs=10)  # TODO tune this
        self.drive_motor.enableCurrentLimit(True)
        self.drive_motor.enableVoltageCompensation(True)

    def nt_offset_changed(self, entry, key: str, value: float, flags: int):
        value = int(value)
        self.steer_enc_offset = value
        self.steer_motor.configSetCustomParam(value, 0, timeoutMs=10)

    def store_steer_offsets(self):
        """Store the current steer positions as the offsets."""
        pos = self.steer_motor.getSelectedSensorPosition(0)
        self.steer_enc_offset = pos
        self.steer_enc_offset_entry.setDouble(pos)
        self.steer_motor.configSetCustomParam(pos, 0, timeoutMs=10)

    def reset_encoder_delta(self):
        """Re-zero the encoder deltas as returned from get_encoder_delta.

        This is intended to be called by the SwerveChassis in order to track
        odometry.
        """
        self.zero_azimuth = self.read_azimuth()
        self.zero_drive_pos = self.wheel_pos

    def get_encoder_delta(self):
        """Return the difference between the modules' current position and
        their position at the last time reset_encoder_delta was called.
        This is intended to be called by the SwerveChassis in order to track
        odometry.
        """
        steer_delta = constrain_angle(self.read_azimuth() - self.zero_azimuth)
        drive_delta = self.wheel_pos - self.zero_drive_pos
        return steer_delta, drive_delta

    def get_cartesian_delta(self):
        """Return the [x, y] position deltas for this module since the last
        time reset_encoder_delta was called.
        This is intended to be called by the SwerveChassis in order to track
        odometry.
        """
        azimuth_delta, drive_delta = self.get_encoder_delta()

        avg_azimuth = self.read_azimuth() - (azimuth_delta / 2)

        if abs(azimuth_delta) > 0.0001:
            # correct for the fact that when we are rotating the modules move in
            # arcs when we are rotating, not straight lines
            drive_delta = (
                2 * math.sin(azimuth_delta / 2) * (drive_delta / azimuth_delta)
            )

        drive_x_delta = drive_delta * math.cos(avg_azimuth)
        drive_y_delta = drive_delta * math.sin(avg_azimuth)

        return (drive_x_delta, drive_y_delta)

    def get_cartesian_vel(self):
        azimuth = self.read_azimuth()
        drive_speed = self.wheel_vel

        drive_x_vel = drive_speed * math.cos(azimuth)
        drive_y_vel = drive_speed * math.sin(azimuth)
        return drive_x_vel, drive_y_vel

    def set_velocity(self, vx: float, vy: float, *, absolute_rotation: bool = False):
        """Set the x and y components of the desired module velocity, relative
        to the robot.

        :param vx: desired x velocity, m/s (x is forward on the robot)
        :param vy: desired y velocity, m/s (y is left on the robot)
        """
        measured_azimuth = self.read_azimuth()
        speed = math.hypot(vx, vy)
        if speed == 0:
            if self.last_speed != 0:
                self.drive_motor.setIntegralAccumulator(0, 0, 0)
                self.drive_motor.neutralOutput()
                self.steer_motor.neutralOutput()
                self.last_speed = 0
            return

        self.last_speed = speed
        desired_azimuth = math.atan2(vy, vx)

        if absolute_rotation:
            # Calculate a delta to from the module's current setpoint (wrapped
            # to between +-pi), representing required rotation to get to our
            # desired angle
            delta = constrain_angle(desired_azimuth - measured_azimuth)
        else:
            # figure out the most efficient way to get the module to the desired direction
            delta = self.min_angular_displacement(measured_azimuth, desired_azimuth)
        # Following commented block is to work with the (currently broken) PID
        # control on the Talon SRXs themselves.
        # Please note, this is *NOT WRAPPED* to +-pi, because if wrapped the module
        # will unwind
        azimuth_to_set = measured_azimuth + delta
        # convert the direction to encoder counts to set as the closed-loop setpoint
        self.setpoint = (
            azimuth_to_set * self.STEER_COUNTS_PER_RADIAN + self.steer_enc_offset
        )
        self.steer_motor.set(ctre.ControlMode.MotionMagic, self.setpoint)
        # self.steer_motor.set(ctre.ControlMode.Position, self.setpoint)

        if not absolute_rotation:
            # logic to only move the modules when we are close to the corret angle
            azimuth_error = constrain_angle(measured_azimuth - desired_azimuth)
            if abs(azimuth_error) < math.pi / 4:
                # if we are nearing the correct angle with the module forwards
                self.drive_motor.setNeutralMode(ctre.NeutralMode.Brake)
                self.drive_motor.set(
                    ctre.ControlMode.Velocity,
                    speed * self.drive_velocity_to_native_units,
                )
                self.aligned = True
            elif abs(azimuth_error) > math.pi - math.pi / 4:
                # if we are nearing the correct angle with the module backwards
                self.drive_motor.setNeutralMode(ctre.NeutralMode.Brake)
                self.drive_motor.set(
                    ctre.ControlMode.Velocity,
                    -speed * self.drive_velocity_to_native_units,
                )
                self.aligned = True
            else:
                self.drive_motor.setNeutralMode(ctre.NeutralMode.Coast)
                self.drive_motor.set(ctre.ControlMode.Velocity, 0)
                self.aligned = False
        else:
            self.drive_motor.set(
                ctre.ControlMode.Velocity, speed * self.drive_velocity_to_native_units
            )

    def update_odometry(self):
        drive_pos = self.drive_motor.getSelectedSensorPosition(0)
        drive_vel = self.drive_motor.getSelectedSensorVelocity(0)
        self.wheel_vel = drive_vel / self.drive_velocity_to_native_units
        self.wheel_angular_vel = drive_vel / self.drive_angular_vel_to_native_units
        self.wheel_pos = drive_pos / self.drive_counts_per_metre

    def read_azimuth(self):
        steer_pos = self.steer_motor.getSelectedSensorPosition(0)
        return (steer_pos - self.steer_enc_offset) / self.STEER_COUNTS_PER_RADIAN

    @staticmethod
    def min_angular_displacement(current, target):
        """Get the minimum (signed) angular displacement from `current` to `target` in radians."""
        opp_target = constrain_angle(target + math.pi)
        diff = constrain_angle(target - current)
        opp_diff = constrain_angle(opp_target - current)

        if abs(diff) < abs(opp_diff):
            return diff
        return opp_diff

    def set_drive_coast(self):
        self.drive_motor.setNeutralMode(ctre.NeutralMode.Coast)

    def set_drive_brake(self):
        self.drive_motor.setNeutralMode(ctre.NeutralMode.Brake)
