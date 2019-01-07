import magicbot
import wpilib


class Robot(magicbot.MagicRobot):
    def createObjects(self):
        """Create motors and stuff here"""
        pass

    def teleopInit(self):
        """Called when teleop starts; optional"""
        pass

    def teleopPeriodic(self):
        """Called on each iteration of the control loop"""
        pass


if __name__ == "__main__":
    wpilib.run(Robot)
