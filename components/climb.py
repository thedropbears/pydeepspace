import rev
import wpilib
import ctre
from utilities.pid import PID


class Lift:
    motor: rev.CANSparkMax

    limit_switch: wpilib.DigitalInput

    GROUND_OFFSET = 5  # motor rotations

    # TODO get values
    # heights in metres
    LIFT_METRES_PER_REV = 0.002

    EXTENDED_HEIGHT = 5
    RETRACTED_HEIGHT = 5

    LIFT_SPEED = 5676  # rpm

    THRESHOLD = 0.001

    up_PID_slot = 0
    up_PID = PID(0, 0, 0, 0)

    down_PID_slot = 1
    down_PID = PID(0, 0, 0, 0)

    def setup(self):
        self.lift_encoder = self.motor.getEncoder()
        self.lift_pid_controller = self.motor.getPIDController()
        self.lift_limit_switch = self.motor.getForwardLimitSwitch(
            rev.LimitSwitchPolarity.kNormallyOpen
        )

        self.set_pid(self.up_PID, self.up_PID_slot)
        self.set_pid(self.down_PID, self.down_PID_slot)

        self.lift_limit_switch.enableLimitSwitch(True)
        self.motor.setIdleMode(rev.IdleMode.kBrake)

        self.set_point = self.lift_encoder.getPosition()

    def set_pid(self, pid, slot):
        self.lift_pid_controller.setP(pid.P, slotID=slot)
        self.lift_pid_controller.setI(pid.I, slotID=slot)
        self.lift_pid_controller.setD(pid.D, slotID=slot)
        self.lift_pid_controller.setFF(pid.F, slotID=slot)

    def retract(self):
        self.set_lift_height_metres(self.RETRACTED_HEIGHT)

    def extend(self):
        self.set_lift_height_metres(self.EXTENDED_HEIGHT)

    def is_at_set_pos(self):
        lift_pos = self.lift_encoder.getPosition()
        return self.is_within_threshold(lift_pos, self.set_point)

    def stop(self):
        self.set_point = self.lift_encoder.getPosition()
        self.motor.stopMotor()

    def is_touching_podium(self):
        return self.limit_switch.get()

    def set_lift_height_metres(self, set_point_metres):
        if set_point_metres - self.get_lift_height_metres() > 0:
            self.current_pid_slot = self.up_PID_slot
        else:
            self.current_pid_slot = self.down_PID_slot

        self.set_point = (
            set_point_metres / self.LIFT_METRES_PER_REV + self.GROUND_OFFSET
        )

    def get_lift_height_metres(self):
        pos = self.lift_encoder.getPosition()
        return (pos - self.GROUND_OFFSET) * self.LIFT_METRES_PER_REV

    def is_within_threshold(self, pos, goal):
        return (goal + self.THRESHOLD) >= pos >= (goal - self.THRESHOLD)

    def execute(self):
        if not self.is_at_set_pos():
            self.lift_pid_controller.setReference(
                self.set_point, rev.ControlType.kPosition, pidSlot=self.current_pid_slot
            )

    def on_disable(self):
        self.stop()


class LiftDrive:
    motor: ctre.TalonSRX

    DRIVE_ENCODER_TYPE = ctre.FeedbackDevice.QuadEncoder

    DRIVE_SPEED = 13180 / 600  # rpm to counts/100ms

    drive_PID = PID(0, 0, 0, 0)

    drive_wheels = False

    def setup(self):
        self.motor.configSelectedFeedbackSensor(
            self.DRIVE_ENCODER_TYPE, 0, timeoutMs=10
        )

        self.motor.config_kP(0, self.drive_PID.P, timeoutMs=10)
        self.motor.config_kI(0, self.drive_PID.I, timeoutMs=10)
        self.motor.config_kD(0, self.drive_PID.D, timeoutMs=10)
        self.motor.config_kF(0, self.drive_PID.F, timeoutMs=10)

        self.motor.setNeutralMode(ctre.NeutralMode.Brake)

    def move(self):
        self.drive_wheels = True

    def stop(self):
        self.motor.set(ctre.ControlMode.PercentOutput, 0)
        self.drive_wheels = False

    def execute(self):
        if self.drive_wheels:
            self.motor.set(ctre.ControlMode.PercentOutput, 1)

    def on_disable(self):
        self.stop()
