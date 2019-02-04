#!/usr/bin/env python3
import math

import ctre

# import rev
import magicbot
import wpilib
from networktables import NetworkTables

from automations.alignment import HatchDepositAligner, HatchIntakeAligner, CargoDepositAligner
from automations.cargo import CargoManager
from components.cargo import Arm, Intake
from components.hatch import Hatch
from automations.climb import ClimbAutomation
from components.vision import Vision
from components.climb import Lift, LiftDrive
from pyswervedrive.chassis import SwerveChassis
from pyswervedrive.module import SwerveModule
from utilities.functions import constrain_angle, rescale_js
from utilities.navx import NavX


class Robot(magicbot.MagicRobot):
    # Declare magicbot components here using variable annotations.
    # NOTE: ORDER IS IMPORTANT.
    # Any components that actuate objects should be declared after
    # any higher-level components (automations) that depend on them.

    # Automations
    cargo: CargoManager
    cargo_deposit: CargoDepositAligner
    climb_automation: ClimbAutomation
    hatch_deposit: HatchDepositAligner
    hatch_intake: HatchIntakeAligner

    # Actuators
    arm: Arm
    chassis: SwerveChassis
    hatch: Hatch
    intake: Intake

    front_lift: Lift
    back_lift: Lift
    lift_drive: LiftDrive

    offset_rotation_rate = 20

    def createObjects(self):
        """Create motors and stuff here."""

        # a + + b - + c - - d + -
        x_dist = 0.2625
        y_dist = 0.2165
        self.module_a = SwerveModule(  # front right module
            "a",
            steer_talon=ctre.TalonSRX(7),
            drive_talon=ctre.TalonSRX(8),
            x_pos=x_dist,
            y_pos=y_dist,
        )
        self.module_b = SwerveModule(  # top left module
            "b",
            steer_talon=ctre.TalonSRX(1),
            drive_talon=ctre.TalonSRX(2),
            x_pos=-x_dist,
            y_pos=y_dist,
        )
        self.module_c = SwerveModule(  # bottom left module
            "c",
            steer_talon=ctre.TalonSRX(3),
            drive_talon=ctre.TalonSRX(4),
            x_pos=-x_dist,
            y_pos=-y_dist,
            reverse_drive_direction=False,
            reverse_drive_encoder=True
        )
        self.module_d = SwerveModule(  # bottom right module
            "d",
            steer_talon=ctre.TalonSRX(5),
            drive_talon=ctre.TalonSRX(6),
            x_pos=x_dist,
            y_pos=-y_dist,
        )
        self.imu = NavX()
        self.vision = Vision()

        self.sd = NetworkTables.getTable("SmartDashboard")
        wpilib.SmartDashboard.putData("Gyro", self.imu.ahrs)

        # hatch objects
        self.hatch_bottom_puncher = wpilib.Solenoid(0)
        self.hatch_left_puncher = wpilib.Solenoid(1)
        self.hatch_right_puncher = wpilib.Solenoid(2)

        self.hatch_top_limit_switch = wpilib.DigitalInput(1)
        self.hatch_left_limit_switch = wpilib.DigitalInput(2)
        self.hatch_right_limit_switch = wpilib.DigitalInput(3)
        # self.front_lift_motor = rev.CANSparkMax(0, rev.MotorType.kBrushless)
        self.front_lift_limit_switch = wpilib.DigitalInput(4)

        # self.back_lift_motor = rev.CANSparkMax(1, rev.MotorType.kBrushless)
        self.back_lift_limit_switch = wpilib.DigitalInput(5)

        self.lift_drive_motor = ctre.TalonSRX(20)

        # cargo related objects
        self.intake_motor = ctre.TalonSRX(9)
        self.intake_switch = wpilib.DigitalInput(0)

        # boilerplate setup for the joystick
        self.joystick = wpilib.Joystick(0)

        self.spin_rate = 1.5

    def disabledPeriodic(self):
        self.chassis.set_inputs(0, 0, 0)
        self.imu.resetHeading()

    def teleopInit(self):
        """Initialise driver control."""
        self.chassis.set_inputs(0, 0, 0)

    def teleopPeriodic(self):
        """Allow the drivers to control the robot."""
        # self.chassis.heading_hold_off()

        throttle = (1 - self.joystick.getThrottle()) / 2
        self.sd.putNumber("joy_throttle", throttle)

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

        self.sd.putNumber("joy_vx", joystick_vx)
        self.sd.putNumber("joy_vy", joystick_vy)
        self.sd.putNumber("joy_vz", joystick_vz)

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
            if self.cargo.has_cargo:
                constrained_angle = -constrain_angle(math.radians(joystick_hat) + math.pi)
            else:
                constrained_angle = -constrain_angle(math.radians(joystick_hat))
            self.chassis.set_heading_sp(constrained_angle)
        
        if self.joystick.getRawButtonPressed(4):
            self.hatch.punch()

        if self.joystick.getTrigger():
            if self.hatch.has_hatch:
                self.hatch_deposit.engage()
            elif self.intake.has_cargo:
                self.cargo_deposit.engage()
            else:
                self.hatch_intake.engage()

        if self.joystick.getRawButtonPressed(5):
            self.hatch.clear_to_retract = True

    def robotPeriodic(self):
        super().robotPeriodic()

        self.sd.putNumber("odometry_x", self.chassis.position[0])
        self.sd.putNumber("odometry_y", self.chassis.position[1])
        for module in self.chassis.modules:
            self.sd.putNumber(
                module.name + "_pos_steer",
                module.steer_motor.getSelectedSensorPosition(0),
            )
            self.sd.putNumber(
                module.name + "_pos_drive",
                module.drive_motor.getSelectedSensorPosition(0),
            )
            self.sd.putNumber(
                module.name + "_drive_motor_reading",
                module.drive_motor.getSelectedSensorVelocity(0)
                * 10  # convert to seconds
                / module.drive_counts_per_metre,
            )
    
    def testPeriodic(self):
        joystick_vx = -rescale_js(
            self.joystick.getY(), deadzone=0.1, exponential=1.5, rate=0.5
        )
        self.sd.putNumber("joy_vx", joystick_vx)

        if self.joystick.getRawButton(5):
            self.module_a.store_steer_offsets()
            self.module_a.steer_motor.set(ctre.ControlMode.PercentOutput, joystick_vx)
            if self.joystick.getTriggerPressed():
                self.module_a.steer_motor.set(
                    ctre.ControlMode.Position,
                    self.module_a.steer_motor.getSelectedSensorPosition(0)
                    + self.offset_rotation_rate,
                )
            if self.joystick.getRawButtonPressed(2):
                self.module_a.steer_motor.set(
                    ctre.ControlMode.Position,
                    self.module_a.steer_motor.getSelectedSensorPosition(0)
                    - self.offset_rotation_rate,
                )

        if self.joystick.getRawButton(3):
            self.module_b.store_steer_offsets()
            self.module_b.steer_motor.set(ctre.ControlMode.PercentOutput, joystick_vx)
            if self.joystick.getTriggerPressed():
                self.module_b.steer_motor.set(
                    ctre.ControlMode.Position,
                    self.module_b.steer_motor.getSelectedSensorPosition(0)
                    + self.offset_rotation_rate,
                )
            if self.joystick.getRawButtonPressed(2):
                self.module_b.steer_motor.set(
                    ctre.ControlMode.Position,
                    self.module_b.steer_motor.getSelectedSensorPosition(0)
                    - self.offset_rotation_rate,
                )

        if self.joystick.getRawButton(4):
            self.module_c.store_steer_offsets()
            self.module_c.steer_motor.set(ctre.ControlMode.PercentOutput, joystick_vx)
            if self.joystick.getTriggerPressed():
                self.module_c.steer_motor.set(
                    ctre.ControlMode.Position,
                    self.module_c.steer_motor.getSelectedSensorPosition(0)
                    + self.offset_rotation_rate,
                )
            if self.joystick.getRawButtonPressed(2):
                self.module_c.steer_motor.set(
                    ctre.ControlMode.Position,
                    self.module_c.steer_motor.getSelectedSensorPosition(0)
                    - self.offset_rotation_rate,
                )

        if self.joystick.getRawButton(6):
            self.module_d.store_steer_offsets()
            self.module_d.steer_motor.set(ctre.ControlMode.PercentOutput, joystick_vx)
            if self.joystick.getTriggerPressed():
                self.module_d.steer_motor.set(
                    ctre.ControlMode.Position,
                    self.module_d.steer_motor.getSelectedSensorPosition(0)
                    + self.offset_rotation_rate,
                )
            if self.joystick.getRawButtonPressed(2):
                self.module_d.steer_motor.set(
                    ctre.ControlMode.Position,
                    self.module_d.steer_motor.getSelectedSensorPosition(0)
                    - self.offset_rotation_rate,
                )

        if self.joystick.getRawButtonPressed(8):
            for module in self.chassis.modules:
                module.drive_motor.set(ctre.ControlMode.PercentOutput, 0.3)

        if self.joystick.getRawButtonPressed(12):
            for module in self.chassis.modules:
                module.steer_motor.set(
                    ctre.ControlMode.Position, module.steer_enc_offset
                )
        
        if self.joystick.getRawButton(11):
            for module in self.chassis.modules:
                module.drive_motor.set(ctre.ControlMode.PercentOutput, 0.3)


if __name__ == "__main__":
    wpilib.run(Robot)
