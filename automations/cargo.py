from magicbot import StateMachine, state, timed_state

from components.cargo import CargoManipulator, Height
from components.vision import Vision


class CargoManager(StateMachine):

    cargo_component: CargoManipulator
    vision: Vision

    def on_disable(self):
        self.done()

    def intake_floor(self, force=False):
        self.engage(initial_state="move_to_floor", force=force)

    @state(first=True, must_finish=True)
    def move_to_floor(self, initial_call, state_tm):
        self.cargo_component.move_to(Height.FLOOR)
        self.cargo_component.intake()
        self.next_state("intaking_cargo")

    def outake_cargo_ship(self, force=False):
        self.engage(initial_state="move_to_cargo_ship", force=force)

    @state(must_finish=True)
    def move_to_cargo_ship(self, initial_call, state_tm):
        self.cargo_component.move_to(Height.CARGO_SHIP)
        if self.cargo_component.at_height(Height.CARGO_SHIP):
            self.next_state("outtaking_cargo")

    def intake_loading(self, force=False):
        self.engage(initial_state="move_to_loading_station", force=force)

    @state(must_finish=True)
    def move_to_loading_station(self, initial_call, state_tm):
        self.cargo_component.move_to(Height.LOADING_STATION)
        self.cargo_component.intake()
        self.next_state("intaking_cargo")

    @state(must_finish=True)
    def intaking_cargo(self):
        self.vision.use_cargo()
        if self.cargo_component.is_contained():
            self.next_state("finishing_intake")
        else:
            self.cargo_component.intake()

    @state(must_finish=True)
    def outtaking_cargo(self, initial_call, state_tm):
        self.cargo_component.outtake()

        if state_tm > 1:
            self.vision.use_hatch()
            self.done()

    @timed_state(duration=1)
    def finishing_intake(self):
        self.cargo_component.slow_intake()

    def done(self):
        self.cargo_component.stop()
        self.cargo_component.move_to(Height.LOADING_STATION)
        super().done()
