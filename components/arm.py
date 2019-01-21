import wpilib
import magicbot


class Arm:

    bot_extension: wpilib.Solenoid
    top_extension: wpilib.Solenoid

    def __init__(self):
        self.bot_extension_on = None
        self.top_extension_on = None
        self.last_bot_extension_on = None
        self.last_top_extension_on = None

    def setup(self):
        pass

    def execute(self):
        if self.bot_extension_on != self.last_bot_extension_on:
            self.bot_extension.set(self.bot_extension_on)
        if self.top_extension_on != self.last_top_extension_on:
            self.top_extension.set(self.top_extension_on)

    def raise_bot_ext(self):
        self.bot_extension_on = True

    def lower_bot_ext(self):
        self.bot_extension_on = False

    def toggle_bot_ext(self):
        self.bot_extension_on = not self.bot_extension_on

    def raise_top_ext(self):
        self.top_extension_on = True

    def lower_top_ext(self):
        self.top_extension_on = False

    def toggle_top_ext(self):
        self.top_extension_on = not self.top_extension_on

    @magicbot.feedback
    def bot_ext_pos(self):
        return self.bot_extension.get()

    @magicbot.feedback
    def top_ext_pos(self):
        return self.top_extension.get()
