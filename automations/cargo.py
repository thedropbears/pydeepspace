from magicbot import state

from automations.align import AlignBase
from components.arm import Arm
from components.intake import Intake


class CargoManager(AlignBase):

    arm: Arm
    intake: Intake

    def start_intake(self, force=False):
        self.engage(initial_state="lower_floor", force=force)

    @state(first=True)
    def lower_floor(self):
        self.arm.lower_bot_ext()
        self.arm.lower_top_ext()
        self.next_state("intaking_cargo")

    @state(must_finish=True)
    def intaking_cargo(self):
        self.intake.intake()
        if self.intake.contained():
            self.intake.stop()
            self.done()

    def outtake_cargo_rocket(self, force=False):
        self.engage(initial_state="raise_rocket", force=force)

    @state
    def raise_rocket(self):
        self.arm.lower_top_ext()
        self.arm.raise_bot_ext()
        self.next_state_now("aligning")

    def outtake_cargo_ship(self, force=False):
        self.engage(initial_state="raise_cargo", force=force)

    @state
    def raise_cargo(self):
        self.arm.raise_top_ext()
        self.arm.raise_bot_ext()
        self.next_state_now("aligning")

    @state
    def successful_align(self):
        self.next_state_now("outtaking_cargo")

    @state
    def failed_align(self):
        self.done()

    @state(must_finish=True)
    def outtaking_cargo(self):
        self.intake.outtake()
        if not self.intake.contained():
            self.done()
