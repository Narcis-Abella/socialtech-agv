"""The /scan of the running simulation (robot still), measured the way lidar_noise_from_scan.py measured the real one, against calibration.json. usage: scan_selfcheck.py [SECONDS=60] [calibration.json]
Needs the realism nodes (or the legacy noise nodes, to see what they gave): per beam with a return in >= 50 % of the frames, over the whole stretch: core sigma (frames within 10 cm of the beam's median), outlier rate, dropout;
binned by the beam's median range. The sim world is not the real room: the BINS present depend on the geometry; the numbers per bin must match."""
import json
import sys
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

T = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
cal = json.load(open(sys.argv[2] if len(sys.argv) > 2 else "/tools/sim_realism/calibration.json"))["lidar_scan"]
rclpy.init(); n = rclpy.create_node("scan_selfcheck"); frames = []
n.create_subscription(LaserScan, "/scan", lambda m: frames.append(np.array(m.ranges, dtype=float)), qos_profile_sensor_data)
t0 = time.time()
while time.time() - t0 < T: rclpy.spin_once(n, timeout_sec=0.1)
X = np.array(frames); print(f"{len(X)} scans of {X.shape[1]} beams in {T:.0f} s")
v = np.isfinite(X) & (X > 0.05); p = v.mean(0); ok = p >= 0.5; Xm = np.where(v, X, np.nan); med = np.nanmedian(Xm[:, ok], 0); dev = Xm[:, ok] - med
core = np.where(np.abs(dev) < 0.10, dev, np.nan); core_sd = np.nanstd(core, 0); outl = np.isfinite(dev) & (np.abs(dev) >= 0.10); q = outl.sum(0) / np.maximum(np.isfinite(dev).sum(0), 1); drop = 1 - p[ok]
bins = cal["range_bins_m"] + [60.0]; print("range (m)   | beams | core sigma median (mm)  sim | cal | outlier rate  sim | cal | dropout  sim | cal")
bad = 0
for i, (lo, hi) in enumerate(zip(bins[:-1], bins[1:])):
    if i >= len(cal["core_sigma_m"]): break
    m = (med >= lo) & (med < hi)
    if m.sum() < 30: continue
    cs, orr, dr = np.nanmedian(core_sd[m]) * 1000, q[m].mean(), drop[m].mean()
    flag = "" if abs(cs - cal["core_sigma_m"][i] * 1000) < 6 and abs(dr - cal["dropout_mean"][i]) < 0.04 and abs(orr - cal["outlier_rate_mean"][i]) < 0.08 else "  <-- off"
    bad += bool(flag); print(f"{lo:5.1f}-{hi:5.1f} | {int(m.sum()):5d} | {cs:6.1f} | {cal['core_sigma_m'][i]*1000:6.1f} | {orr:5.3f} | {cal['outlier_rate_mean'][i]:5.3f} | {dr:5.3f} | {cal['dropout_mean'][i]:5.3f}{flag}")
sys.exit(1 if bad else 0)
