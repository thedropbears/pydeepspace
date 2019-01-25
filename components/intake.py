import ctre
import wpilib


class Intake:

    motor: ctre.TalonSRX
    switch: wpilib.DigitalInput

    def __init__(self):
        self.last_motor_output = None
        self.motor_output = 0

    def execute(self):
        if self.motor_output != self.last_motor_output:
            self.motor.set(ctre.ControlMode.PercentOutput, self.motor_output)
            self.last_motor_output = self.motor_output

    def intake(self):
        self.motor_output = -1

    def outtake(self):
        self.motor_output = 1

    def stop(self):
        self.motor_output = 0

    def emergency_stop(self):
        self.motor.stopMotor()

    def contained(self):
        return self.switch.get()
