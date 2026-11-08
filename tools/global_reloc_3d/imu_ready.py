"""Online S3 readiness from the robot's own IMU (truth-free).

FAST-LIO2 takes the mean gyro of its first samples as gyro bias and the mean
accelerometer as gravity (IMU_Processing.hpp:156-196). Starting it while the robot
turns absorbs the turn rate into the bias. This gate starts a cold FAST-LIO2 only
after a 2 s IMU window passes the independently calibrated stationarity test of
06-enmienda-estacionariedad-imu.md, unchanged: p95(|gyro - median|) <= 0.006119479
rad/s, |median| <= 0.027834677 rad/s, max |gyro| <= 0.05 rad/s, SI accelerometer
sample-variance sum <= 0.04 (m/s^2)^2, >= 95 % of the expected samples.
"""

import numpy as np

G = 9.80665
WINDOW_S = 2.
RESIDUAL_P95 = 0.006119479213085752
MEDIAN_NORM = 0.0278346772285473
RAW_MAX = .05
ACC_VAR_SUM = .04
# The start test above is calibrated to decide whether a FILTER or an EXPOSURE may start. "Was it moved?" (during the exposure and the search) is another question: the Nano's gyro is
# noisier with the stack running (p95 0.006-0.009 rad/s, 2026-11-07, IMU of the Nano parked 36 min) and that noise is not motion. A shove shows in max|gyro| and in the accelerometer variance,
# a creep in the median, none of which this scale touches; real motion measured 0.45 rad/s of noise (median, 26 % of a 21 min drive), 50 times the limit.
MOTION_RESIDUAL_SCALE = 3.


def expected_samples(t, span_s=None):
    """How many samples a gap-free stream would have in the window: span / typical period. The typical period is the mean of the steps under 3x the median step: a dropout is not the rate,
    and the jitter of the Livox IMU on the Nano (long tail: median 4.5 ms, mean 5.15 ms) did not inflate the expectation to 444 for a window that holds 388, as span / median(dt) did."""
    t = np.asarray(t, float)
    d = np.diff(t)
    d = d[d > 0]                                    # a stamp that repeats or steps back is not a period
    if len(d) < 2:
        return 0
    return int(np.ceil((t[-1] - t[0] if span_s is None else span_s) / d[d < 3 * np.median(d)].mean())) if t[-1] > t[0] or span_s is not None else 0


def window_ok(gyro, acc_g, expected, residual_scale=1.):
    gyro = np.asarray(gyro, float)
    if expected < 3 or len(gyro) < .95 * expected or len(gyro) < 3:      # expected <= 0 is a stream whose stamps tell nothing: not still, never 'no requirement'
        return False
    b = np.median(gyro, axis=0)
    residual = np.percentile(np.linalg.norm(gyro - b, axis=1), 95)
    acc_var = np.var(np.asarray(acc_g, float) * G, axis=0, ddof=1).sum()
    return bool(residual <= RESIDUAL_P95 * residual_scale and np.linalg.norm(b) <= MEDIAN_NORM
                and np.linalg.norm(gyro, axis=1).max() <= RAW_MAX and acc_var <= ACC_VAR_SUM)
