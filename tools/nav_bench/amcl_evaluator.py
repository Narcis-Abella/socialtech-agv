"""Post-run truth scoring. Keep this module out of executor/method containers."""
import math


def classify(accepted, xy_error_m=None, yaw_error_rad=None, negative=False):
    """Return the point-4 primary class from a completed attempt and evaluator truth."""
    if accepted:
        if xy_error_m is None or yaw_error_rad is None:
            raise ValueError('accepted trial needs evaluator-computed errors')
        if not math.isfinite(xy_error_m) or not math.isfinite(yaw_error_rad):
            raise ValueError('accepted trial errors must be finite')
        wrapped_yaw=math.atan2(math.sin(yaw_error_rad),math.cos(yaw_error_rad))
        return ('correct_accept' if xy_error_m<=.30 and abs(wrapped_yaw)<=math.radians(8)
                else 'wrong_accept')
    return 'correct_reject' if negative else 'false_reject_timeout'
