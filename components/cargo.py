import enum
import math

import ctre
import rev
import wpilib

from components.vision import Vision


class Height(enum.Enum):
    FLOOR = 18.6
    CARGO_SHIP = 0
    LOADING_STATION = 0


class CargoManipulator:

    vision: Vision

    arm_motor: rev.CANSparkMax
    intake_motor: ctre.VictorSPX

    intake_switch: wpilib.DigitalInput

    GEAR_RATIO = 7 * 5 * 84 / 50
    UNITS_PER_RADIAN = 18.6 / math.radians(105)  # measured

    INTAKE_SPEED = -0.75
    SLOW_INTAKE_SPEED = -0.2
    OUTTAKE_SPEED = 1.0

    def __init__(self):
        self.intake_motor_output = 0.0

    def setup(self) -> None:
        self.arm_motor.setIdleMode(rev.IdleMode.kBrake)
        self.arm_motor.setInverted(False)

        self.intake_motor.setNeutralMode(ctre.NeutralMode.Brake)

        self.encoder = self.arm_motor.getEncoder()

        self.pid_controller = self.arm_motor.getPIDController()
        self.pid_controller.setP(5e-4)
        self.pid_controller.setI(1e-6)
        self.pid_controller.setD(0)
        self.pid_controller.setIZone(0)
        self.pid_controller.setFF(1 / 5675)
        self.pid_controller.setOutputRange(-1, 1)
        self.pid_controller.setSmartMotionMaxVelocity(1200)  # rpm
        self.pid_controller.setSmartMotionMaxAccel(1000)  # rpm/s
        self.pid_controller.setSmartMotionAllowedClosedLoopError(0)
        self.pid_controller.setOutputRange(-1, 1)

        self.top_limit_switch = self.arm_motor.getReverseLimitSwitch(
            rev.LimitSwitchPolarity.kNormallyOpen
        )
        self.bottom_limit_switch = self.arm_motor.getForwardLimitSwitch(
            rev.LimitSwitchPolarity.kNormallyOpen
        )
        self.top_limit_switch.enableLimitSwitch(True)
        self.bottom_limit_switch.enableLimitSwitch(True)

        self.setpoint = Height.LOADING_STATION.value
        self.tolerance = 0.1
        self.has_cargo = False

    def execute(self) -> None:
        self.intake_motor.set(ctre.ControlMode.PercentOutput, self.intake_motor_output)
        self.pid_controller.setReference(self.setpoint, rev.ControlType.kSmartMotion)

        if self.is_contained():
            self.has_cargo = True
            self.vision.use_cargo()

        if self.top_limit_switch.get():
            self.encoder.setPosition(Height.LOADING_STATION.value)
        if self.bottom_limit_switch.get():
            self.encoder.setPosition(Height.FLOOR.value)

    def at_height(self, desired_height) -> bool:
        return abs(desired_height.value - self.encoder.getPosition()) <= self.tolerance

    def move_to(self, height: Height) -> None:
        """Move arm to specified height.

        Args:
            height: Height to move arm to
        """
        self.setpoint = height.value

    def on_disable(self) -> None:
        self.intake_motor.set(ctre.ControlMode.PercentOutput, 0)
        self.arm_motor.set(0)

    def on_enable(self) -> None:
        self.setpoint = self.encoder.getPosition()

    def intake(self) -> None:
        self.intake_motor_output = self.INTAKE_SPEED

    def outtake(self) -> None:
        self.has_cargo = False
        self.intake_motor_output = self.OUTTAKE_SPEED

    def slow_intake(self) -> None:
        self.intake_motor_output = self.SLOW_INTAKE_SPEED

    def stop(self) -> None:
        self.intake_motor_output = 0

    def is_contained(self) -> bool:
        return not self.intake_switch.get()
