import math


def rescale_js(
    value: float, deadzone: float = 0, exponential: float = 0, rate: float = 1
) -> float:
    """Rescale a joystick input, applying a deadzone, exponential, and max rate.

    Args:
        value: the joystick value, in the interval [-1, 1].
        deadzone: the deadzone to apply.
        exponential: the strength of the exponential to apply
                     (i.e. how non linear should the response be)
        rate: the max rate to return (i.e. the value to be returned when 1 is given)
    """
    sign = 1
    if value < 0:
        sign = -1
        value = -value
    # Apply deadzone
    if value < deadzone:
        return 0
    if not exponential:
        value = (value - deadzone) / (1 - deadzone)
    else:
        a = math.log(exponential + 1) / (1 - deadzone)
        value = (math.exp(a * (value - deadzone)) - 1) / exponential
    return rate * sign * value


def constrain_angle(angle: float) -> float:
    """Wrap an angle to the interval [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))
