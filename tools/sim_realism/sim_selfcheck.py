"""End-to-end check of the realism models inside the running simulation. usage: sim_selfcheck.py   (sim + sim_realism.launch.py running, SIM_REALISM=1, odom_source:=ground_truth, Nav2 NOT running)
Drives /cmd_vel with a fixed profile and compares /odom_gt_raw (the true twist of the model) and /odom (what the wheel odometry reports) with the command, fitting the same model the calibration found on the real base:
  linear v: dead time, first-order lag, gain; angular w moving; angular w in place: deadband; wheel odometry: reported / true, quantisation.
Prints a table 'measured | calibration' and exits 1 when something is off by more than the tolerance."""
import json
import math
import os
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

PROFILE = [(0.0, 0.0, 2.0), (0.6, 0.0, 5.0), (0.0, 0.0, 3.0), (-0.3, 0.0, 4.0), (0.0, 0.0, 2.0),
           (0.0, 0.10, 4.0), (0.0, 0.30, 4.0), (0.0, 0.60, 4.0), (0.0, -0.60, 4.0), (0.0, 0.0, 2.0), (0.3, 0.4, 5.0), (0.3, -0.4, 5.0), (0.0, 0.0, 2.0)]   # (v, w, seconds)
rclpy.init(); n = rclpy.create_node("sim_selfcheck", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)])   # all times in simulation time: the real-time factor is not 1
CMD_TOPIC = os.environ.get("CMD_TOPIC", "/cmd_vel")   # /cmd_vel_gz: Gazebo alone, without sim_base_model
now = lambda: n.get_clock().now().nanoseconds * 1e-9
cmd_pub = n.create_publisher(Twist, CMD_TOPIC, 10); gt, rep, cmdlog = [], [], []
n.create_subscription(Odometry, "/odom_gt_raw", lambda m: gt.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.twist.twist.linear.x, m.twist.twist.angular.z)), 50)
n.create_subscription(Odometry, "/odom", lambda m: rep.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.twist.twist.linear.x, m.twist.twist.angular.z)), 50)
while now() == 0.0: rclpy.spin_once(n, timeout_sec=0.1)   # wait for /clock
t_wall0, t_sim0 = time.time(), now()
for v, w, dur in PROFILE:
    seg_end = now() + dur
    while now() < seg_end:
        m = Twist(); m.linear.x = v; m.angular.z = w; cmd_pub.publish(m); cmdlog.append((now(), v, w)); rclpy.spin_once(n, timeout_sec=0.01)
print(f"real-time factor during the check: {(now() - t_sim0) / (time.time() - t_wall0):.2f}"); gt, rep, cmdlog = (np.array(a) for a in (gt, rep, cmdlog)); dt = 0.02; g = np.arange(max(gt[0, 0], cmdlog[0, 0]) + 0.5, min(gt[-1, 0], cmdlog[-1, 0]) - 0.5, dt)
U = np.c_[np.interp(g, cmdlog[:, 0], cmdlog[:, 1]), np.interp(g, cmdlog[:, 0], cmdlog[:, 2])]; Y = np.c_[np.interp(g, gt[:, 0], gt[:, 1]), np.interp(g, gt[:, 0], gt[:, 2])]; R = np.c_[np.interp(g, rep[:, 0], rep[:, 1]), np.interp(g, rep[:, 0], rep[:, 2])]


def lowpass(x, tau):
    a = dt / (tau + dt); y = np.empty_like(x); y[0] = x[0]
    for i in range(1, len(x)): y[i] = y[i - 1] + a * (x[i] - y[i - 1])
    return y


def fit(u, y, mask):
    best = None
    for Td in np.arange(0, 0.41, 0.02):
        sh = int(round(Td / dt)); us = np.roll(u, sh); us[:sh] = 0
        for tau in (0.0, 0.03, 0.05, 0.075, 0.1, 0.15, 0.2):
            x = lowpass(us, tau); A, B = x[mask], y[mask]; G = (A @ B) / (A @ A); rms = math.sqrt(np.mean((B - G * A) ** 2))
            if best is None or rms < best[0]: best = (rms, Td, tau, G)
    return best[1:]


cal = json.load(open("/tools/sim_realism/calibration.json")) if len(sys.argv) < 2 else json.load(open(sys.argv[1]))
fails = []
def row(name, got, want, tol):
    ok = abs(got - want) <= tol; fails.append(not ok); print(f"{'ok  ' if ok else 'FAIL'} {name:46s} measured {got:8.3f} | calibration {want:8.3f} (tol {tol})")


mv_v = (np.abs(U[:, 0]) > 0.05) | (np.abs(np.roll(U[:, 0], 40)) > 0.05); Td, tau, G = fit(U[:, 0], Y[:, 0], mv_v)
row("linear dead time (s)", Td, cal["base_response"]["linear"]["dead_time_s"], 0.04); row("linear lag tau (s)", tau, cal["base_response"]["linear"]["lowpass_tau_s"], 0.05); row("linear gain", G, cal["base_response"]["linear"]["gain"], 0.03)
mv_w = (np.abs(U[:, 0]) > 0.2) & (np.abs(U[:, 1]) > 0.2); Td, tau, G = fit(U[:, 1], Y[:, 1], mv_w)
row("angular moving gain", G, cal["base_response"]["angular_moving"]["gain"], 0.05); row("angular moving dead time (s)", Td, cal["base_response"]["angular_moving"]["dead_time_s"], 0.06)
for amp in (0.10, 0.30, 0.60):
    seg = (np.abs(np.abs(U[:, 1]) - amp) < 1e-9) & (np.abs(U[:, 0]) < 0.01); idx = np.where(seg)[0][int(1.5 / dt):]   # skip the transient
    if len(idx): print(f"     in place |w cmd| {amp:.2f}: true {np.mean(np.abs(Y[idx, 1])):.3f} rad/s -> deadband estimate {amp - np.mean(np.abs(Y[idx, 1])) / 0.97:.3f} rad/s (this run's draw is in sim_base_model's log)")
seg = (np.abs(U[:, 0]) > 0.5); row("wheel odom linear reported/true", float((R[seg, 0] @ Y[seg, 0]) / (Y[seg, 0] @ Y[seg, 0])), cal["wheel_odometry"]["linear_reported_over_actual"], 0.012)
seg = (np.abs(U[:, 0]) < 0.01) & (np.abs(U[:, 1]) > 0.5); got = float((R[seg, 1] @ Y[seg, 1]) / (Y[seg, 1] @ Y[seg, 1])); row("wheel odom angular reported/true, in place", got, 0.94, 0.05)
vals = np.unique(np.round(rep[:, 1], 6)); print(f"     distinct reported linear values near 0.3: {vals[(np.abs(vals) > 0.25) & (np.abs(vals) < 0.35)][:6]} (step {cal['wheel_odometry']['linear_quantisation_m_s']})")
print("RESULT", "ALL OK" if not any(fails) else "FAILURES"); sys.exit(1 if any(fails) else 0)
