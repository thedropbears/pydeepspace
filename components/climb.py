import math
import wpilib
import wpilib_controller
import rev
import ctre
from utilities.navx import NavX
from dataclasses import dataclass


@dataclass
class Lift:
    motor: rev.CANSparkMax
    encoder: rev._impl.CANEncoder
    pid_controller: rev._impl.CANPIDController
    forward_limit_switch: rev._impl.CANDigitalInput


class Climber:
    front_motor: rev.CANSparkMax
    back_motor: rev.CANSparkMax

    drive_motor: ctre.TalonSRX

    front_podium_switch: wpilib.DigitalInput
    back_podium_switch: wpilib.DigitalInput

    solenoid: wpilib.DoubleSolenoid

    imu: NavX

    LIFT_SPEED = 1  # 1500  # 0.5  # 4700/5840 rpm
    front_direction = 0
    back_direction = 0

    DRIVE_SPEED = 0.4
    drive_wheels = False

    HEIGHT_PER_REV = 0.002
    GROUND_CLEARANCE = -0.05

    SLOW_DOWN_SPEED = 0.15

    def setup(self):
        self.drive_motor.setNeutralMode(ctre.NeutralMode.Brake)
        self.drive_motor.setInverted(True)

        self.lifts = []
        for lift in [self.front_motor, self.back_motor]:
            self.lifts.append(
                Lift(
                    lift,
                    lift.getEncoder(),
                    lift.getPIDController(),
                    lift.getForwardLimitSwitch(rev.LimitSwitchPolarity.kNormallyOpen),
                )
            )

        for lift in self.lifts:
            lift.motor.setIdleMode(rev.IdleMode.kBrake)
            lift.pid_controller.setP(0.1)
            lift.pid_controller.setOutputRange(-1, 1)
            lift.forward_limit_switch.enableLimitSwitch(True)

        self.front_lift = self.lifts[0]
        self.back_lift = self.lifts[1]

        self.front_lift.reverse_limit_switch = self.front_lift.motor.getReverseLimitSwitch(
            rev.LimitSwitchPolarity.kNormallyOpen
        )
        self.front_lift.reverse_limit_switch.enableLimitSwitch(True)

        self.level_pid = wpilib_controller.PIDController(
            Kp=3, Ki=0, Kd=0, period=1 / 50, measurement_source=self.imu.getPitch
        )
        self.level_pid.setInputRange(-math.pi, math.pi)
        self.level_pid.setOutputRange(-1, 1)
        self.level_pid.setReference(0)

        self.level_pid_enabled = True

        wpilib.SmartDashboard.putData("lift_level_pid", self.level_pid)
        wpilib.SmartDashboard.putData(
            "lift_front_podium_switch", self.front_podium_switch
        )
        wpilib.SmartDashboard.putData(
            "lift_back_podium_switch", self.back_podium_switch
        )

    def extend_all(self):
        self.extend_front()
        self.extend_back()

    def retract_all(self):
        self.retract_front()
        self.retract_back()

    def retract_front(self):
        self.front_direction = 1

    def extend_front(self):
        self.front_direction = -1

    def retract_back(self):
        self.back_direction = 1

    def extend_back(self):
        self.back_direction = -1

    def is_both_extended(self):
        return self.front_lift.reverse_limit_switch.get()

    def is_front_retracted(self):
        return self.front_lift.forward_limit_switch.get()

    def is_front_above_ground_level(self):
        return (
            self.front_lift.encoder.getPosition()
            > self.GROUND_CLEARANCE / self.HEIGHT_PER_REV
        )

    def is_back_above_ground_level(self):
        return (
            self.back_lift.encoder.getPosition()
            > self.GROUND_CLEARANCE / self.HEIGHT_PER_REV
        )

    def is_back_retracted(self):
        return self.back_lift.forward_limit_switch.get()

    def is_front_touching_podium(self):
        return self.front_podium_switch.get()

    def is_back_touching_podium(self):
        return not self.back_podium_switch.get()

    def stop_front(self):
        self.front_direction = 0

    def stop_back(self):
        self.back_direction = 0

    def stop_all(self):
        self.stop_front()
        self.stop_back()
        self.stop_wheels()

    def execute(self):
        for lift in self.lifts:
            if lift.forward_limit_switch.get():
                lift.encoder.setPosition(0)

        # Extend both
        if self.front_direction < 0 and self.back_direction < 0:
            pid_output = self.level_pid.update()  # * self.LIFT_SPEED

            if self.front_lift.reverse_limit_switch.get():
                self.front_lift.motor.set(0)
                self.back_lift.motor.set(0)

            else:
                self.front_lift.motor.set(-self.LIFT_SPEED + pid_output)
                self.back_lift.motor.set(-self.LIFT_SPEED - pid_output)

        # Retract both
        elif self.front_direction > 0 and self.back_direction > 0:
            output = self.LIFT_SPEED * 0.4

            pid_output = self.level_pid.update()  # * self.LIFT_SPEED
            if self.front_lift.encoder.getPosition() > self.SLOW_DOWN_THRESHOLD:
                self.front_lift.motor.set(self.SLOW_DOWN_SPEED)
            else:
                self.front_lift.motor.set(output + pid_output)

            if self.back_lift.encoder.getPosition() > self.SLOW_DOWN_THRESHOLD:
                self.back_lift.motor.set(self.SLOW_DOWN_SPEED)
            else:
                self.back_lift.motor.set(output - pid_output)

        # Retract front
        elif self.front_direction > 0:
            output = self.LIFT_SPEED

            if self.is_front_above_ground_level():
                self.front_lift.motor.set(self.SLOW_DOWN_SPEED)
            else:
                self.front_lift.motor.set(output)

            self.back_lift.motor.set(0)

        # Retract back
        elif self.back_direction > 0:
            output = self.LIFT_SPEED

            if self.is_back_above_ground_level():
                self.back_lift.motor.set(self.SLOW_DOWN_SPEED)
            else:
                self.back_lift.motor.set(output)

            self.front_lift.motor.set(0)
        else:
            self.front_lift.motor.set(0)
            self.back_lift.motor.set(0)

        if self.drive_wheels:
            self.drive_motor.set(ctre.ControlMode.PercentOutput, self.DRIVE_SPEED)
        else:
            self.drive_motor.set(ctre.ControlMode.PercentOutput, 0)

    def on_disable(self):
        self.stop_all()
        self.front_lift.motor.set(0)
        self.back_lift.motor.set(0)

    def on_enable(self):
        self.retract_solenoid()

    def move_wheels(self):
        self.drive_wheels = True

    def stop_wheels(self):
        self.drive_wheels = False

    def fire_solenoid(self):
        self.solenoid.set(wpilib.DoubleSolenoid.Value.kForward)

    def retract_solenoid(self):
        self.solenoid.set(wpilib.DoubleSolenoid.Value.kReverse)
