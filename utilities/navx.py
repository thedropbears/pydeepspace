import math

import navx


class NavX:
    """Wrapper around RobotPy NavX to match our coordinate system."""

    def __init__(self):
        self.ahrs = navx.AHRS.create_spi(update_rate_hz=50)

    def getAngle(self) -> float:
        """Get the current yaw in radians.

        Angles are in the interval [-pi, pi], anticlockwise positive.
        """
        raw = self.ahrs.getYaw()
        return -math.radians(raw)

    def getRoll(self) -> float:
        """Get current roll in radians (rotation around Y axis)

        Angles are in the interval [-pi, pi], anticlockwise positive.
        """
        raw = self.ahrs.getRoll()
        return -math.radians(raw)

    def getPitch(self) -> float:
        """Get current pitch in radians (rotation around X axis)

        Angles are in the interval [-pi, pi], anticlockwise positive.
        """
        raw = self.ahrs.getPitch()
        return -math.radians(raw)

    def resetHeading(self):
        """Zero the yaw."""
        self.ahrs.reset()

    def getHeadingRate(self):
        """Get the rate of change of yaw in radians per second."""
        # multiply by update freq because NavX does not normalise per timestep
        return math.radians(-self.ahrs.getRate()) * 50
