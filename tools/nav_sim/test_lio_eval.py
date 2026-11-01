import math

import numpy as np

import lio_eval as le


def track(n=50, offset=(3., -2., .7), drift_xy=0., drift_yaw=0.):
    """A GT arc and a LIO track that is the same arc in another frame (rotated/translated), optionally drifting."""
    rows = []
    for k in range(n):
        s = k * .2
        gx, gy, gyaw = 2 + 3 * math.sin(s / 3), 1 + 3 * (1 - math.cos(s / 3)), s / 3
        c, sn = math.cos(offset[2]), math.sin(offset[2])
        # the LIO frame is the truth frame moved by `offset`: lio = R^-1 (g - t)
        dx, dy = gx - offset[0], gy - offset[1]
        lx, ly = c * dx + sn * dy, -sn * dx + c * dy
        rows.append([k * .1, lx + drift_xy * s, ly, .01 * k * 0, le.wrap(gyaw - offset[2] + drift_yaw * s), gx, gy, 0., gyaw])
    return rows


def test_a_track_that_is_the_truth_in_another_frame_has_no_error():
    e = le.errors(track())
    assert e['xy'].max() < 1e-9 and np.abs(e['yaw']).max() < 1e-9 and np.abs(e['z']).max() < 1e-12


def test_drift_is_what_the_report_sees():
    e = le.errors(track(drift_xy=.01, drift_yaw=.002))
    assert 0.05 < e['xy'][-1] < 0.5
    assert math.degrees(e['yaw'][-1]) > 0.5
    s = le.summary(e)
    assert s['xy_m']['final'] > s['xy_m']['median']


def test_yaw_error_wraps():
    rows = track()
    rows[-1][4] = rows[-1][4] + 2 * math.pi
    assert abs(le.errors(rows)['yaw'][-1]) < 1e-9
