import ctre
import wpilib

class Intake:
  
  intake_motor: ctre.TalonSRX

  def __init__(self):
    self.last_motor_output = None
    self.motor_output = None
  
  def setup(self):
    pass
  
  def execute(self):
    if self.motor_output != self.last_motor_output:
        self.intake_motor.set(ctre.ControlMode.PercentOutput, self.motor_output)
    self.last_motor_output = self.motor_output

  def intake(self):
    self.motor_output = -1

  def outtake(self):
    self.motor_output = 1
  
  def toggle(self):
    if self.motor_output:
      self.motor_output *= -1
    else:
      self.motor_output = -1
  
  def stop(self):
    self.motor_output = 0
  
  def emergency_stop(self):
    self.intake_motor.stopMotor()