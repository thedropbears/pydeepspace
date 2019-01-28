from components.hatch import Hatch
from magicbot import StateMachine


class HatchController(StateMachine):
    hatch: Hatch

    def start_match(self):
        """Initialise the hatch system at the start of the match."""
        self.engage()