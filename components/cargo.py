import enum

import ctre
import wpilib


class Intake:

    motor: ctre.TalonSRX
    intake_switch: wpilib.DigitalInput

    def __init__(self):
        self.motor_output = 0
        self.has_cargo = False

    def execute(self):
        self.motor.set(ctre.ControlMode.PercentOutput, self.motor_output)
        if not self.intake_switch.get():
            self.has_cargo = True

    def intake(self):
        self.motor_output = -1

    def outtake(self):
        self.motor_output = 1
        self.has_cargo = False

    def stop(self):
        self.motor_output = 0


class Height(enum.Enum):
    FLOOR = 0
    ROCKET_SHIP = 42
    CARGO_SHIP = 42
    LOADING_STATION = 42


class Arm:
    def execute(self):
        pass

    def move_to(self, height: Height):
        pass
