"""Real base: command -> actual motion, and wheel-odometry measurement, from a nav bag. usage: base_response_from_bag.py BAG_DIR
Inputs: /cmd_vel (Twist, what the base receives), /odom (wheel odometry: reported twist), /livox/imu (gyro z = actual yaw rate, rad/s; bias removed at rest), /tf odom->base_footprint (FAST-LIO planar pose: actual forward speed
from its derivative over +-0.25 s). Everything is resampled to 50 Hz. Motion segments only (|cmd| or |actual| above a threshold, dilated 1 s).
1. Response of the base to its command, per axis: y(t) = G * lowpass_tau(u(t - Td)): grid search of Td, tau, G (least squares) over the motion; reported with the residual RMS and R^2.
2. Wheel odometry (reported twist) against the actual motion: gain (regression), lag (cross-correlation peak), residual noise RMS after the gain, noise at rest, quantisation step (smallest spacing of distinct reported values)."""
import math
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

bag = sys.argv[1]
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
ty = {k.name: k.type for k in r.get_all_topics_and_types()}; names = ["/cmd_vel", "/odom", "/livox/imu", "/tf"]; r.set_filter(rosbag2_py.StorageFilter(topics=names)); M = {k: get_message(ty[k]) for k in names if k in ty}
yaw = lambda q: math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
cmd, odo, imu, pose = [], [], [], []
while r.has_next():
    tp, raw, tn = r.read_next(); t = tn / 1e9; m = deserialize_message(raw, M[tp])
    if tp == "/cmd_vel": cmd.append((t, m.linear.x, m.angular.z))
    elif tp == "/odom": odo.append((t, m.twist.twist.linear.x, m.twist.twist.angular.z))
    elif tp == "/livox/imu": imu.append((t, m.angular_velocity.z))
    else:
        for x in m.transforms:
            if (x.header.frame_id, x.child_frame_id) == ("odom", "base_footprint"): pose.append((t, x.transform.translation.x, x.transform.translation.y, yaw(x.transform.rotation)))
cmd, odo, imu, pose = (np.array(a) for a in (cmd, odo, imu, pose))
t0 = max(a[0, 0] for a in (cmd, odo, imu, pose)); t1 = min(a[-1, 0] for a in (cmd, odo, imu, pose)); dt = 0.02; g = np.arange(t0, t1, dt)
U = np.c_[np.interp(g, cmd[:, 0], cmd[:, 1]), np.interp(g, cmd[:, 0], cmd[:, 2])]; W = np.c_[np.interp(g, odo[:, 0], odo[:, 1]), np.interp(g, odo[:, 0], odo[:, 2])]
gz = np.interp(g, imu[:, 0], imu[:, 1]); py = np.unwrap(pose[:, 3]); px, pyy = np.interp(g, pose[:, 0], pose[:, 1]), np.interp(g, pose[:, 0], pose[:, 2]); pth = np.interp(g, pose[:, 0], py)
k = int(0.25 / dt); vx = (np.roll(px, -k) - np.roll(px, k)) / (2 * k * dt); vy = (np.roll(pyy, -k) - np.roll(pyy, k)) / (2 * k * dt); v_act = vx * np.cos(pth) + vy * np.sin(pth); v_act[:k] = v_act[-k:] = 0
rest = (np.abs(W[:, 0]) < 0.005) & (np.abs(W[:, 1]) < 0.01) & (np.abs(U[:, 0]) < 0.005) & (np.abs(U[:, 1]) < 0.005); gz = gz - np.median(gz[rest])   # gyro bias at rest
w_act = gz; Y = np.c_[v_act, w_act]
mv = (np.abs(U[:, 0]) > 0.03) | (np.abs(U[:, 1]) > 0.05) | (np.abs(Y[:, 0]) > 0.03) | (np.abs(Y[:, 1]) > 0.05); mv = np.convolve(mv.astype(float), np.ones(int(1.0 / dt)), mode="same") > 0; mv[:2 * k] = False
print(f"{(t1-t0):.0f} s of bag, {mv.sum()*dt:.0f} s of motion, {rest.sum()*dt:.0f} s at rest (gyro bias removed: {np.median(imu[:,1]) :+.4f} rad/s raw)")


def lowpass(x, tau):
    if tau < dt: return x.copy()
    a = dt / (tau + dt); y = np.empty_like(x); y[0] = x[0]
    for i in range(1, len(x)): y[i] = y[i - 1] + a * (x[i] - y[i - 1])
    return y


def fit(u, y, mask, label, gain_split=None):
    best = None
    for Td in np.arange(0.0, 0.81, 0.04):
        sh = int(round(Td / dt)); us = np.roll(u, sh); us[:sh] = 0
        for tau in (0.0, 0.05, 0.1, 0.15, 0.2, 0.3, 0.4, 0.6, 0.8):
            x = lowpass(us, tau); A = x[mask]; B = y[mask]; G = float((A @ B) / max(A @ A, 1e-12)); res = B - G * A; rms = math.sqrt(np.mean(res ** 2))
            if best is None or rms < best[0]: best = (rms, Td, tau, G)
    rms, Td, tau, G = best; r2 = 1 - rms ** 2 / np.var(y[mask])
    print(f"   {label}: dead time {Td*1000:.0f} ms, first-order tau {tau*1000:.0f} ms, gain {G:.3f}; residual RMS {rms:.4f}, R2 {r2:.3f}")
    return Td, tau, G


print("\n1. base response to /cmd_vel (actual: FAST-LIO derivative for v, gyro z for w)")
for ax, nm in ((0, "linear v"), (1, "angular w")):
    fit(U[:, ax], Y[:, ax], mv, nm)
big = mv & (np.abs(U[:, 1]) > 0.25); small = mv & (np.abs(U[:, 1]) <= 0.25) & (np.abs(U[:, 1]) > 0.05)
fit(U[:, 1], Y[:, 1], big, "angular w, |cmd| > 0.25 rad/s"); fit(U[:, 1], Y[:, 1], small, "angular w, 0.05 < |cmd| <= 0.25 rad/s")
fit(U[:, 0], Y[:, 0], mv & (U[:, 0] > 0.05), "linear v, forward only")

print("\n2. wheel odometry (reported /odom twist) against the actual motion")
for ax, nm, unit in ((0, "linear v", "m/s"), (1, "angular w", "rad/s")):
    x = W[:, ax]; y = Y[:, ax]; lags = np.arange(-int(0.6 / dt), int(0.6 / dt) + 1); cc = []; idx = np.where(mv)[0]
    for L in lags:
        ii = idx + L; ok = (ii >= 0) & (ii < len(x)); cc.append(np.corrcoef(y[idx[ok]], x[ii[ok]])[0, 1])
    L = lags[int(np.argmax(cc))]; ii = np.clip(idx + L, 0, len(x) - 1); G = float((x[ii] @ y[idx]) / (x[ii] @ x[ii])); res = y[idx] - G * x[ii]
    vals = np.unique(np.round(odo[:, 1 + ax], 6)); step = np.min(np.diff(vals[np.abs(vals) < 0.5])) if len(vals) > 3 else float("nan")
    print(f"   {nm}: reported lags actual by {-L*dt*1000:+.0f} ms (negative = reported later), actual = {G:.3f} x reported (reported/actual = {1/G:.3f}); residual RMS {np.sqrt(np.mean(res**2)):.4f} {unit}; at rest the report has std {W[rest, ax].std():.5f} {unit}; smallest spacing of distinct reported values {step:.5f} {unit}")
# integrated: path length and turned angle of the wheel odometry against FAST-LIO and gyro
vs_ = np.cumsum(np.abs(W[mv, 0])) * dt; vf = np.cumsum(np.abs(v_act[mv])) * dt
turn_w = np.sum(W[mv, 1]) * dt; turn_g = np.sum(gz[mv]) * dt; turn_abs_w = np.sum(np.abs(W[mv, 1])) * dt; turn_abs_g = np.sum(np.abs(gz[mv])) * dt
print(f"\n3. integrated over the motion: path length wheels {vs_[-1]:.1f} m vs FAST-LIO {vf[-1]:.1f} m (ratio {vs_[-1]/vf[-1]:.3f}); total |turn| wheels {math.degrees(turn_abs_w):.0f} deg vs gyro {math.degrees(turn_abs_g):.0f} deg (ratio {turn_abs_w/turn_abs_g:.3f}); net turn wheels {math.degrees(turn_w):+.0f} vs gyro {math.degrees(turn_g):+.0f} deg")
