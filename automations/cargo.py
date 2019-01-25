from magicbot import state, StateMachine

from automations.align import Aligner
from components.arm import Arm
from components.intake import Intake


class CargoManager(StateMachine):

    arm: Arm
    intake: Intake
    align: Aligner

    def __init__(self):
        self.override = False

    def intake_floor(self, force=False):
        self.engage(initial_state="move_to_floor", force=force)

    @state(first=True, must_finish=True)
    def move_to_floor(self):
        self.arm.lower_top_piston()
        self.arm.lower_bottom_piston()
        self.next_state("intaking_cargo")

    def intake_depot(self, force=False):
        self.engage(initial_state="move_to_depot", force=force)

    @state
    def move_to_depot(self):
        self.arm.lower_bottom_piston()
        self.arm.lower_top_piston()
        self.next_state("intaking_cargo")

    def intake_loading(self, force=False):
        self.engage(initial_state="move_to_loading_station", force=force)

    @state
    def move_to_loading_station(self):
        self.arm.raise_top_piston()
        self.arm.raise_bottom_piston()
        self.next_state("intaking_cargo")

    @state(must_finish=True)
    def intaking_cargo(self):
        if self.intake.contained():
            self.intake.stop()
            self.arm.raise_bottom_piston()
            self.arm.lower_top_piston()
            self.done()
        else:
            self.intake.intake()

    def start_outtake(self, force=False):
        self.engage(initial_state="outtaking_cargo", force=force)

    @state(must_finish=True)
    def outtaking_cargo(self):
        if not self.override:
            self.align.align()
        self.intake.outtake()
        if not self.intake.contained():
            self.intake.stop()
            self.done()
