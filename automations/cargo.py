from magicbot import state

from automations.align import AlignBase
from components.arm import Arm
from components.intake import Intake


class CargoManager(AlignBase):

    arm: Arm
    intake: Intake

    def move_to_cargo_ship(self, force=False):
        self.engage(initial_state="moving_cargo_ship", force=force)

    @state(first=True)
    def moving_cargo_ship(self):
        self.arm.raise_bot_ext()
        self.arm.raise_top_ext()
        self.done()

    def move_to_rocket_ship(self, force=False):
        self.engage(initial_state="moving_rocket_ship", force=force)

    @state
    def moving_rocket_ship(self):
        self.arm.raise_bot_ext()
        self.arm.lower_top_ext()
        self.done()

    def move_to_floor(self, force=False):
        self.engage(initial_state="moving_floor", force=force)

    @state
    def moving_floor(self):
        self.arm.lower_bot_ext()
        self.arm.lower_top_ext()

    def start_intake(self, force):
        self.engage(initial_state="aligning", force=force)

    @state
    def successful_align(self):
        print("successful align")
        self.next_state("intaking_cargo")

    @state
    def failed_align(self):
        print("failed align")
        self.done()

    @state(must_finish=True)
    def intaking_cargo(self):
        self.intake.intake()
        if self.intake.contained():
            self.intake.stop_intake()
            self.done()
