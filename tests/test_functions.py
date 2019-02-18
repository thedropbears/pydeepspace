from utilities.functions import rotate_vector

import math


def test_rotate_vector():
    x, y = rotate_vector(2, 1, math.pi / 2)
    assert abs(x - -1) < 1e-4
    assert abs(y - 2) < 1e-4

    x, y = rotate_vector(2, 1, math.pi)
    assert abs(x - -2) < 1e-4
    assert abs(y - -1) < 1e-4
