import math

from networktables.util import ntproperty


class Vision:
    target_tape_error = ntproperty("/vision/target_tape_error", math.nan)
    within_deposit_range = ntproperty("/vision/within_deposit_range", False)
    ground_tape_x = ntproperty("/vision/ground_tape_x", math.nan)
    ground_tape_y = ntproperty("/vision/ground_tape_y", math.nan)
    ground_tape_angle = ntproperty("/vision/ground_tape_angle", math.nan)
    # NOTE: x and y are relative to the robot co-ordinate system, not the camera

    def get_target_tape_error(self):
        if not -1 <= self.target_tape_error <= 1:
            return None
        else:
            return self.target_tape_error

    def get_ground_tape_x(self):
        if not -1 <= self.ground_tape_x <= 1:
            return None
        else:
            return self.ground_tape_x

    def get_ground_tape_y(self):
        if not -1 <= self.ground_tape_y <= 1:
            return None
        else:
            return self.ground_tape_y

    def get_ground_tape_angle(self):
        if not -1 <= self.ground_tape_angle <= 1:
            return None
        else:
            return self.ground_tape_angle
