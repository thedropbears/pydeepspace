import math


def rescale_js(
    value, deadzone: float = 0.0, exponential: float = 0.0, rate: float = 1.0
):
    """Rescale the joystick inputs, applying a deadzone, exponential, and max rate.

    Args:
        value: the joystick value, between + and - 1.
        deadzone: the deadzone to apply.
        exponential: the strength of the exponential to apply (ie how non
        linear should the response be)
        rate: the max rate to return (ie the value to be returned when 1.0 is given)
    Returns:
        The joystick value.
    """
    value_negative = 1.0
    if value < 0:
        value_negative = -1.0
        value = -value
    # Cap to be +/-1
    if abs(value) > 1.0:
        value /= abs(value)
    # Apply deadzone
    if abs(value) < deadzone:
        return 0.0
    elif exponential == 0.0:
        value = (value - deadzone) / (1 - deadzone)
    else:
        a = math.log(exponential + 1) / (1 - deadzone)
        value = (math.exp(a * (value - deadzone)) - 1) / exponential
    return value * value_negative * rate


def constrain_angle(angle):
    """Wrap :param angle: to between +pi and -pi"""
    return math.atan2(math.sin(angle), math.cos(angle))
