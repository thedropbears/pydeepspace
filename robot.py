#!/usr/bin/env python3

import ctre
import magicbot
import wpilib

from pyswervedrive.swervechassis import SwerveChassis
from pyswervedrive.swervemodule import SwerveModule
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

        self.imu = NavX()

    def teleopInit(self):
        """Initialise driver control."""
        pass

    def teleopPeriodic(self):
        """Allow the drivers to control the robot."""
        pass


if __name__ == "__main__":
    wpilib.run(Robot)
