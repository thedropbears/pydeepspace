import enum

import ctre
import wpilib


class Intake:

    motor: ctre.TalonSRX
    switch: wpilib.DigitalInput

    def __init__(self):
        self.motor_output = 0

    def execute(self):
        self.motor.set(ctre.ControlMode.PercentOutput, self.motor_output)

    def intake(self):
        self.motor_output = -1

    def outtake(self):
        self.motor_output = 1

    def stop(self):
        self.motor_output = 0

    def is_contained(self):
        return self.switch.get()


class Heights(enum.Enum):
    pass


class Arm:
    def execute(self):
        pass

    def move_to(self, height: Heights):
        pass
