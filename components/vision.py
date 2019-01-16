import hal

from networktables import NetworkTables


class Vision:

    def __init__(self):
        """This is called after variables are injected by magicbot."""
        self.nt = NetworkTables.getTable("/vision")
        self.target_tape_error = self.nt.getEntry("target_tape_error")
        self.ground_tape_error = self.nt.getEntry("ground_tape_error")
        self.target_tape_error.addListener(
            self.new_target_value,
            NetworkTables.NotifyFlags.NEW | NetworkTables.NotifyFlags.UPDATE,
        )
        self.ground_tape_error.addListener(
            self.new_ground_value,
            NetworkTables.NotifyFlags.NEW | NetworkTables.NotifyFlags.UPDATE,
        )

        # network tables doesn't like none, so we use out of bound values
        self.target_tape_error_value = 999
        self.ground_tape_error_value = 999

    def new_target_value(self, entry, key, value, param):
        # self.time = time.monotonic()
        self.target_tape_error_value = value

    def new_ground_value(self, entry, key, value, param):
        # self.time = time.monotonic()
        self.ground_tape_error_value = value

    def get_target_tape_error(self):
        if self.target_tape_error_value < -1 or self.target_tape_error_value > 1:
            return None
        elif hal.isSimulation():
            return 0
        else:
            return self.target_tape_error_value

    def get_ground_tape_error(self):
        return None # disable for testing
        if self.ground_tape_error_value < -1 or self.ground_tape_error_value > 1:
            return None
        elif hal.isSimulation():
            return 0
        else:
            return self.ground_tape_error_value
