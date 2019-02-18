from magicbot import StateMachine, state

from components.cargo import CargoManipulator, Height


class CargoManager(StateMachine):

    cargo_component: CargoManipulator

    def intake_floor(self, force=False):
        self.engage(initial_state="move_to_floor", force=force)

    @state(first=True, must_finish=True)
    def move_to_floor(self, initial_call, state_tm):
        self.cargo_component.move_to(Height.FLOOR)
        if self.cargo_component.at_height(Height.FLOOR):
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
        if self.cargo_component.at_height(Height.LOADING_STATION):
            self.next_state("intaking_cargo")

    @state(must_finish=True)
    def intaking_cargo(self):
        if self.cargo_component.is_contained():
            self.cargo_component.stop()
            self.done()
        else:
            self.cargo_component.intake()

    @state(must_finish=True)
    def outtaking_cargo(self, initial_call, state_tm):
        if initial_call:
            self.cargo_component.outtake()

        if state_tm > 0.5:
            self.cargo_component.stop()
            self.done()
