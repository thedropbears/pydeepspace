import math

import ctre
import magicbot
import rev
import wpilib
import wpilib_controller

from utilities.navx import NavX


class Lift:
    HEIGHT_PER_REV = 0.002
    GROUND_CLEARANCE = -0.05

    __slots__ = ("motor", "encoder", "forward_limit_switch")

    def __init__(self, motor: rev.CANSparkMax) -> None:
        self.motor = motor
        self.encoder = motor.getEncoder()
        self.forward_limit_switch = motor.getForwardLimitSwitch(
            rev.LimitSwitchPolarity.kNormallyOpen
        )

        self.motor.setIdleMode(rev.IdleMode.kBrake)
        self.forward_limit_switch.enableLimitSwitch(True)
        self.encoder.setPositionConversionFactor(self.HEIGHT_PER_REV)
        self.encoder.setVelocityConversionFactor(self.HEIGHT_PER_REV / 60)

    def is_retracted(self) -> bool:
        return self.forward_limit_switch.get()

    def is_above_ground(self) -> bool:
        return self.encoder.getPosition() > self.GROUND_CLEARANCE


class Climber:
    front_motor: rev.CANSparkMax
    back_motor: rev.CANSparkMax

    drive_motor: ctre.TalonSRX

    front_podium_switch: wpilib.DigitalInput
    back_podium_switch: wpilib.DigitalInput

    pistons: wpilib.DoubleSolenoid

    imu: NavX

    LIFT_SPEED = 1  # 1500  # 0.5  # 4700/5840 rpm
    SLOW_DOWN_SPEED = 0.15

    DRIVE_SPEED = 0.6

    front_direction = magicbot.will_reset_to(0)
    back_direction = magicbot.will_reset_to(0)
    drive_output = magicbot.will_reset_to(0)

    def setup(self):
        self.drive_motor.setNeutralMode(ctre.NeutralMode.Brake)
        self.drive_motor.setInverted(True)

        self.front = Lift(self.front_motor)
        self.back = Lift(self.back_motor)
        self.lifts = (self.front, self.back)

        self.front_reverse_limit_switch = self.front_motor.getReverseLimitSwitch(
            rev.LimitSwitchPolarity.kNormallyOpen
        )
        self.front_reverse_limit_switch.enableLimitSwitch(True)

        self.level_pid = wpilib_controller.PIDController(
            Kp=3, Ki=0, Kd=0, period=1 / 50, measurement_source=self.imu.getPitch
        )
        self.level_pid.setInputRange(-math.pi, math.pi)
        self.level_pid.setOutputRange(-1, 1)
        self.level_pid.setReference(0)

        self.level_pid_enabled = True

        # wpilib.SmartDashboard.putData("lift_level_pid", self.level_pid)

    def extend_all(self):
        self.front_direction = -1
        self.back_direction = -1

    def retract_all(self):
        self.retract_front()
        self.retract_back()

    def retract_front(self):
        self.front_direction = 1

    def retract_back(self):
        self.back_direction = 1

    def is_both_extended(self):
        return self.front_reverse_limit_switch.get()

    def is_front_touching_podium(self):
        return not self.front_podium_switch.get()

    def is_back_touching_podium(self):
        return not self.back_podium_switch.get()

    def execute(self):
        for lift in self.lifts:
            if lift.forward_limit_switch.get():
                lift.encoder.setPosition(0)

        # Extend both
        if self.front_direction < 0 and self.back_direction < 0:
            pid_output = self.level_pid.update()

            if self.is_both_extended():
                self.back.motor.disable()
                self.front.motor.disable()

            else:
                self.front.motor.set(-self.LIFT_SPEED + pid_output)
                self.back.motor.set(-self.LIFT_SPEED - pid_output)

        # Retract both
        elif self.front_direction > 0 and self.back_direction > 0:
            output = self.LIFT_SPEED * 0.4

            if self.front.is_above_ground():
                self.front.motor.set(self.SLOW_DOWN_SPEED)
            else:
                self.front.motor.set(output)

            if self.back.is_above_ground():
                self.back.motor.set(self.SLOW_DOWN_SPEED)
            else:
                self.back.motor.set(output)

        else:
            output = self.LIFT_SPEED

            # Retract front
            if self.front_direction > 0:
                if self.front.is_above_ground():
                    self.front.motor.set(self.SLOW_DOWN_SPEED)
                else:
                    self.front.motor.set(output)
            else:
                self.front.motor.disable()

            # Retract back
            if self.back_direction > 0:
                if self.back.is_above_ground():
                    self.back.motor.set(self.SLOW_DOWN_SPEED)
                else:
                    self.back.motor.set(output)
            else:
                self.back.motor.disable()

        self.drive_motor.set(ctre.ControlMode.PercentOutput, self.drive_output)

    def on_disable(self):
        self.front.motor.disable()
        self.back.motor.disable()

    def on_enable(self):
        self.retract_pistons()

    def drive_forward(self, drive_speed=DRIVE_SPEED):
        self.drive_output = drive_speed

    def fire_pistons(self):
        self.pistons.set(wpilib.DoubleSolenoid.Value.kForward)

    def retract_pistons(self):
        self.pistons.set(wpilib.DoubleSolenoid.Value.kReverse)
