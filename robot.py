#!/usr/bin/env python3

import magicbot
import wpilib


class Robot(magicbot.MagicRobot):
    def createObjects(self):
        """Create motors and stuff here."""
        pass

    def teleopInit(self):
        """Initialise driver control."""
        pass

    def teleopPeriodic(self):
        """Allow the drivers to control the robot."""
        pass


if __name__ == "__main__":
    wpilib.run(Robot)
