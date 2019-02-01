# import rev
import wpilib
import ctre


class Lift:
    # motor: rev.CANSparkMax

    limit_switch: wpilib.DigitalInput

    def setup(self):
        pass
        # self.lift_encoder = self.lift_motor.getEncoder()
        # self.lift_pid_controller = self.lift_motor.getPIDController()

    def retract(self):
        pass

    def extend(self):
        pass

    def is_at_set_pos(self):
        pass

    def stop(self):
        pass

    def is_touching_podium(self):
        pass

    def execute(self):
        pass


class LiftDrive:
    motor: ctre.TalonSRX

    def move(self):
        pass

    def stop(self):
        pass

    def execute(self):
        pass
