"""Online S3 readiness from the robot's own IMU (truth-free).

FAST-LIO2 takes the mean gyro of its first samples as gyro bias and the mean
accelerometer as gravity (IMU_Processing.hpp:156-196). Starting it while the robot
turns absorbs the turn rate into the bias. This gate starts a cold FAST-LIO2 only
after a 2 s IMU window passes the independently calibrated stationarity test of
06-enmienda-estacionariedad-imu.md, unchanged: p95(|gyro - median|) <= 0.006119479
rad/s, |median| <= 0.027834677 rad/s, max |gyro| <= 0.05 rad/s, SI accelerometer
sample-variance sum <= 0.04 (m/s^2)^2, >= 95 % of the expected samples.
"""
import argparse
import json

import numpy as np

G = 9.80665
WINDOW_S = 2.
RESIDUAL_P95 = 0.006119479213085752
MEDIAN_NORM = 0.0278346772285473
RAW_MAX = .05
ACC_VAR_SUM = .04


def window_ok(gyro, acc_g, expected):
    gyro = np.asarray(gyro, float)
    if len(gyro) < .95 * expected or len(gyro) < 3:
        return False
    b = np.median(gyro, axis=0)
    residual = np.percentile(np.linalg.norm(gyro - b, axis=1), 95)
    acc_var = np.var(np.asarray(acc_g, float) * G, axis=0, ddof=1).sum()
    return bool(residual <= RESIDUAL_P95 and np.linalg.norm(b) <= MEDIAN_NORM
                and np.linalg.norm(gyro, axis=1).max() <= RAW_MAX and acc_var <= ACC_VAR_SUM)


def ready_times(t, gyro, acc_g, step=.1):
    """Times T (on a `step` grid) whose preceding half-open window [T-2, T) passes."""
    t = np.asarray(t, float)
    cadence = np.median(np.diff(t))
    expected = int(np.ceil(WINDOW_S / cadence))
    out = []
    for T in np.arange(t[0] + WINDOW_S, t[-1], step):
        i, j = np.searchsorted(t, [T - WINDOW_S, T])
        if window_ok(gyro[i:j], acc_g[i:j], expected):
            out.append(float(T))
    return out


def episodes(times, step=.1):
    """Group ready times into stationary episodes [first_ready, last_ready]."""
    groups = []
    for T in times:
        if groups and T - groups[-1][1] <= step * 1.5:
            groups[-1][1] = T
        else:
            groups.append([T, T])
    return groups


def read_imu(bag):
    from pathlib import Path
    from rosbags.highlevel import AnyReader
    t, g, a = [], [], []
    with AnyReader([Path(bag)]) as reader:
        conn = [c for c in reader.connections if c.topic == '/livox/imu']
        for c, _, raw in reader.messages(connections=conn):
            m = reader.deserialize(raw, c.msgtype)
            t.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
            g.append([m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z])
            a.append([m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z])
    order = np.argsort(t)
    return np.asarray(t)[order], np.asarray(g)[order], np.asarray(a)[order]


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--bag', required=True)
    p.add_argument('--out', required=True)
    a = p.parse_args()
    t, g, acc = read_imu(a.bag)
    eps = episodes(ready_times(t, g, acc))
    with open(a.out, 'w') as f:
        json.dump({'criterion': '06-enmienda-estacionariedad-imu.md, unchanged', 'truth_used': False,
                   'imu_span_s': [float(t[0]), float(t[-1])],
                   'episodes': [{'first_ready_s': e[0], 'last_ready_s': e[1]} for e in eps]}, f, indent=2)
    print(len(eps), 'stationary episodes')


if __name__ == '__main__':
    main()
