from magicbot import AutonomousStateMachine

from utilities.pure_pursuit import PurePursuit


class LeftStartAuto(AutonomousStateMachine):
    def __init__(self):
        self.pursuit = PurePursuit(look_ahead=0.2)
