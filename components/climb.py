# import rev
import wpilib
import ctre
from utilities.pid import PID


class Lift:
    # motor: rev.CANSparkMax
    motor: ctre.TalonSRX

    limit_switch: wpilib.DigitalInput

    GROUND_OFFSET = 5  # motor rotations
    REVERSE_LIMIT = 1

    # TODO get values
    # heights in metres
    METRES_PER_REV = 0.002
    COUNTS_PER_REV = 20

    COUNTS_PER_METRE = COUNTS_PER_REV / METRES_PER_REV

    ENCODER_TYPE = ctre.FeedbackDevice.QuadEncoder

    EXTENDED_HEIGHT = 5
    RETRACTED_HEIGHT = 5

    LIFT_SPEED = 5676  # rpm

    THRESHOLD = 0.001

    up_PID_slot = 0
    up_PID = PID(0, 0, 0, 0)

    down_PID_slot = 1
    down_PID = PID(0, 0, 0, 0)

    def setup(self):
        # self.lift_encoder = self.motor.getEncoder()
        # self.lift_pid_controller = self.motor.getPIDController()
        # self.lift_limit_switch = self.motor.getForwardLimitSwitch(
        #     rev.LimitSwitchPolarity.kNormallyOpen
        # )

        # self.lift_limit_switch.enableLimitSwitch(True)
        # self.motor.setIdleMode(rev.IdleMode.kBrake)

        self.motor.configSelectedFeedbackSensor(self.ENCODER_TYPE, 0, timeoutMs=10)
        self.motor.configForwardLimitSwitchSource(
            ctre.LimitSwitchSource.FeedbackConnector,
            ctre.LimitSwitchNormal.NormallyOpen,
        )

        self.motor.configReverseSoftLimitThreshold(self.REVERSE_LIMIT, timeoutMs=10)
        self.motor.configReverseSoftLimitEnable(True, timeoutMs=10)

        self.motor.setNeutralMode(ctre.NeutralMode.Brake)

        self.set_pid(self.up_PID, self.up_PID_slot)
        self.set_pid(self.down_PID, self.down_PID_slot)

        self.set_point = False

    def set_pid(self, pid, slot):
        # self.lift_pid_controller.setP(pid.P, slotID=slot)
        # self.lift_pid_controller.setI(pid.I, slotID=slot)
        # self.lift_pid_controller.setD(pid.D, slotID=slot)
        # self.lift_pid_controller.setFF(pid.F, slotID=slot)

        self.motor.config_kP(slotIdx=slot, value=pid.P, timeoutMs=10)
        self.motor.config_kI(slotIdx=slot, value=pid.I, timeoutMs=10)
        self.motor.config_kD(slotIdx=slot, value=pid.D, timeoutMs=10)
        self.motor.config_kF(slotIdx=slot, value=pid.F, timeoutMs=10)

    def retract(self):
        self.set_lift_height_metres(self.RETRACTED_HEIGHT)

    def extend(self):
        self.set_lift_height_metres(self.EXTENDED_HEIGHT)

    def get_pos(self):
        # return self.lift_encoder.getPosition()
        return self.motor.getSelectedSensorPosition(0)

    def is_at_set_pos(self):
        lift_pos = self.get_pos()
        return abs(lift_pos - self.set_point) <= self.THRESHOLD

    def stop(self):
        # self.motor.stopMotor()
        self.motor.set(ctre.ControlMode.PercentOutput, 0)

        self.set_point = False

    def is_touching_podium(self):
        return self.limit_switch.get()

    def set_lift_height_metres(self, set_point_metres):
        if set_point_metres - self.get_lift_height_metres() > 0:
            self.current_pid_slot = self.up_PID_slot
        else:
            self.current_pid_slot = self.down_PID_slot

        self.set_point = set_point_metres / self.COUNTS_PER_METRE + self.GROUND_OFFSET

    def get_lift_height_metres(self):
        pos = self.get_pos()
        return (pos - self.GROUND_OFFSET) * self.COUNTS_PER_METRE

    def execute(self):
        if self.set_point:
            # self.lift_pid_controller.setReference(
            #     self.set_point, rev.ControlType.kPosition, pidSlot=self.current_pid_slot
            # )
            self.motor.selectProfileSlot(self.current_pid_slot, 0)
            self.motor.set(ctre.ControlMode.Position, self.set_point)

    def on_disable(self):
        self.stop()


class LiftDrive:
    motor: ctre.TalonSRX

    drive_wheels = False

    def setup(self):
        self.motor.setNeutralMode(ctre.NeutralMode.Brake)

    def move(self):
        self.drive_wheels = True

    def stop(self):
        self.motor.set(ctre.ControlMode.PercentOutput, 0)
        self.drive_wheels = False

    def execute(self):
        if self.drive_wheels:
            self.motor.set(ctre.ControlMode.PercentOutput, 0.2)

    def on_disable(self):
        self.stop()
