"""Deterministic perturbation of the initial pose given to AMCL (stdlib only: it runs inside the sim image, which has no scipy)."""
import math


def init_offset(k, magnitude, yaw_deg):
    """(dx, dy, dyaw) for trial k: `magnitude` metres in a direction that turns 60 deg per trial, `yaw_deg` degrees with alternating sign."""
    a = math.radians(60.0 * k)
    return (magnitude * math.cos(a), magnitude * math.sin(a), math.radians(yaw_deg) * (1 if k % 2 == 0 else -1))
