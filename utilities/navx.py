import math

from navx import AHRS
from wpilib.interfaces import PIDSource


class NavX:
    """Wrapper around RobotPy NavX to be compatible with our BNO055 driver."""

    PIDSourceType = PIDSource.PIDSourceType

    def __init__(self):
        self.ahrs = AHRS.create_spi(update_rate_hz=50)
        self.pidsource = self.PIDSourceType.kDisplacement

    def getAngle(self) -> float:
        """Get NavX angle.

        Returns:
            Angle in radians between -pi and +pi.
        """
        raw = self.ahrs.getYaw()
        return -math.radians(raw)

    def resetHeading(self):
        self.ahrs.reset()

    def getHeadingRate(self):
        # multiply by 100 because NavX does not normalise per timestep
        return math.radians(-self.ahrs.getRate()) * 50

    def pidGet(self):
        if self.pidsource == self.PIDSourceType.kDisplacement:
            return self.getAngle()
        else:
            return self.getHeadingRate()

    def setPIDSourceType(self, pidsourcetype):
        self.pidsource = pidsourcetype

    def getPIDSourceType(self):
        return self.pidsource
