# import rev
import wpilib
import wpilib_controller
import ctre
from utilities.navx import NavX

# from utilities.pid import PID


class Climber:
    front_motor: ctre.TalonSRX
    back_motor: ctre.TalonSRX

    imu: NavX

    LIFT_SPEED = 0.5  # 4700/5840 rpm

    def setup(self):
        self.lifts = [self.front_motor, self.back_motor]
        for lift in self.lifts:
            lift.configForwardLimitSwitchSource(
                ctre.LimitSwitchSource.FeedbackConnector,
                ctre.LimitSwitchNormal.NormallyOpen,
            )
            lift.configReverseLimitSwitchSource(
                ctre.LimitSwitchSource.FeedbackConnector,
                ctre.LimitSwitchNormal.NormallyOpen,
            )

            lift.configPeakCurrentLimit(80, timeoutMs=10)
            lift.configPeakCurrentDuration(500, timeoutMs=10)
            lift.configContinuousCurrentLimit(50, timeoutMs=10)
            lift.enableCurrentLimit(True)

            lift.setNeutralMode(ctre.NeutralMode.Brake)

        self.front_direction = False
        self.back_direction = False

        self.level_pid = wpilib_controller.PIDController(
            Kp=0, Ki=0, Kd=0, period=1 / 50, measurement_source=self.imu.ahrs.getRoll
        )
        self.level_pid.setReference(0)

        self.level_pid_enabled = True

    def extend_all(self):
        self.extend_front()
        self.extend_back()

    def retract_all(self):
        self.retract_front()
        self.retract_back()

    def retract_front(self):
        self.front_direction = -1

    def extend_front(self):
        self.front_direction = 1

    def retract_back(self):
        self.back_direction = -1

    def extend_back(self):
        self.back_direction = 1

    def is_front_at_set_pos(self):
        if self.front_direction == 1:
            return self.front_motor.isFwdLimitSwitchClosed()
        elif self.front_direction == -1:
            return self.front_motor.isRevLimitSwitchClosed()

    def is_back_at_set_pos(self):
        if self.back_direction == 1:
            return self.back_motor.isFwdLimitSwitchClosed()
        elif self.back_direction == -1:
            return self.back_motor.isRevLimitSwitchClosed()

    def stop_front(self):
        self.front_motor.set(ctre.ControlMode.PercentOutput, 0)
        self.front_direction = False

    def stop_back(self):
        self.back_motor.set(ctre.ControlMode.PercentOutput, 0)
        self.back_direction = False

    def stop_all(self):
        self.stop_front()
        self.stop_back()

    def execute(self):
        if self.front_direction:
            output = self.front_direction * self.LIFT_SPEED
            if self.level_pid_enabled:
                output += self.level_pid.update()

            self.front_motor.set(ctre.ControlMode.PercentOutput, output)

        if self.back_direction:
            output = self.back_direction * self.LIFT_SPEED
            if self.level_pid_enabled:
                output += -self.level_pid.update()

            self.back_motor.set(ctre.ControlMode.PercentOutput, output)

    def on_disable(self):
        self.stop_all()


class LiftDrive:
    motor: ctre.TalonSRX

    front_limit_switch: wpilib.DigitalInput
    back_limit_switch: wpilib.DigitalInput

    drive_wheels = False

    DRIVE_SPEED = 0.2

    def setup(self):
        self.motor.setNeutralMode(ctre.NeutralMode.Brake)

    def move(self):
        self.drive_wheels = True

    def stop(self):
        self.motor.set(ctre.ControlMode.PercentOutput, 0)
        self.drive_wheels = False

    def is_front_touching_podium(self):
        return self.front_limit_switch.get()

    def is_back_touching_podium(self):
        return self.back_limit_switch.get()

    def execute(self):
        # if self.is_front_touching_podium or self.is_back_touching_podium():
        #     self.stop()

        if self.drive_wheels:
            self.motor.set(ctre.ControlMode.PercentOutput, self.DRIVE_SPEED)

    def on_disable(self):
        self.stop()
