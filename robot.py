#!/usr/bin/env python3
import math

import ctre
import magicbot
import wpilib

from pyswervedrive.swervechassis import SwerveChassis
from pyswervedrive.swervemodule import SwerveModule
from utilities.functions import constrain_angle, rescale_js
from utilities.navx import NavX


class Robot(magicbot.MagicRobot):
    chassis: SwerveChassis

    module_drive_free_speed: float = 84000.0  # encoder ticks / 100 ms

    def createObjects(self):
        """Create motors and stuff here."""
        self.module_a = SwerveModule(
            "a",
            steer_talon=ctre.TalonSRX(42),
            drive_talon=ctre.TalonSRX(48),
            x_pos=-(0.375 - 0.160),
            y_pos=0.375 - 0.100,
            drive_free_speed=Robot.module_drive_free_speed,
            reverse_steer_encoder=True,
        )
        self.module_b = SwerveModule(
            "b",
            steer_talon=ctre.TalonSRX(58),
            drive_talon=ctre.TalonSRX(2),
            x_pos=0.375 - 0.160,
            y_pos=-(0.375 - 0.100),
            drive_free_speed=Robot.module_drive_free_speed,
            reverse_steer_encoder=True,
        )
        self.module_c = SwerveModule(
            "c",
            steer_talon=ctre.TalonSRX(51),
            drive_talon=ctre.TalonSRX(52),
            x_pos=-(0.375 - 0.160),
            y_pos=-(0.375 - 0.100),
            drive_free_speed=Robot.module_drive_free_speed,
            reverse_steer_encoder=True,
        )
        self.module_d = SwerveModule(
            "d",
            steer_talon=ctre.TalonSRX(53),
            drive_talon=ctre.TalonSRX(54),
            x_pos=0.375 - 0.160,
            y_pos=(0.375 - 0.100),
            drive_free_speed=Robot.module_drive_free_speed,
            reverse_steer_encoder=True,
        )
        self.imu = NavX()

        self.joystick = wpilib.Joystick(0)
        self.spin_rate = 1.5

    def teleopInit(self):
        """Called when teleop starts; optional"""
        self.chassis.set_inputs(0, 0, 0)

    def teleopPeriodic(self):
        """
        Process inputs from the driver station here.
        This is run each iteration of the control loop before magicbot components are executed.
        """
        throttle = (1 - self.joystick.getThrottle()) / 2

        # this is where the joystick inputs get converted to numbers that are sent
        # to the chassis component. we rescale them using the rescale_js function,
        # in order to make their response exponential, and to set a dead zone -
        # which just means if it is under a certain value a 0 will be sent
        # TODO: Tune these constants for whatever robot they are on
        joystick_vx = -rescale_js(
            self.joystick.getY(), deadzone=0.1, exponential=1.5, rate=4 * throttle
        )
        joystick_vy = -rescale_js(
            self.joystick.getX(), deadzone=0.1, exponential=1.5, rate=4 * throttle
        )
        joystick_vz = -rescale_js(
            self.joystick.getZ(), deadzone=0.2, exponential=20.0, rate=self.spin_rate
        )
        joystick_hat = self.joystick.getPOV()

        if joystick_vx or joystick_vy or joystick_vz:
            self.chassis.set_inputs(
                joystick_vx,
                joystick_vy,
                joystick_vz,
                field_oriented=not self.joystick.getRawButton(6),
            )
        else:
            self.chassis.set_inputs(0, 0, 0)

        if joystick_hat != -1:
            constrained_angle = -constrain_angle(math.radians(joystick_hat))
            self.chassis.set_heading_sp(constrained_angle)

    def testPeriodic(self):
        if self.joystick.getRawButtonPressed(6):
            self.module_a.store_steer_offsets()
            self.module_b.store_steer_offsets()
            self.logger.info("offsets changed")


if __name__ == "__main__":
    wpilib.run(Robot)
