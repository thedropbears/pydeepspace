"""
Run tests that come with pyfrc.

This test module imports tests that come with pyfrc, and can be used
to test basic functionality of just about any robot.
"""

# We deliberately avoid importing test_autonomous here,
# because we already test all our autonomous modes (see auto_test.py).
from pyfrc.tests import test_disabled, test_operator_control, test_practice, test_fuzz

# Shut pyflakes up.
__all__ = ("test_disabled", "test_operator_control", "test_practice", "test_fuzz")
