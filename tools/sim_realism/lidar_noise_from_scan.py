"""Range noise and dropout of the REAL /scan as a function of range, from stationary stretches. usage: lidar_noise_from_scan.py BAG_DIR [BAG_DIR ...]  -> prints the table and writes lidar_noise.json next to this file
Stationary = |wheel v| < 0.005 m/s and |w| < 0.01 rad/s for >= 4 s (>= 35 scans). Per beam of a stretch (frames with a return in >= 50 % of the frames): mean range, std of the ranges of the frames that have a return,
dropout = 1 - (share of frames with a return). Bins by mean range. 'stable' std excludes beams that jump between surfaces (std > 0.15 m: reported as the bimodal share)."""
import json
import os
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

BINS = [0.5, 1, 2, 3, 5, 8, 12, 20, 60]
rows = []
for bag in sys.argv[1:]:
    r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
    ty = {k.name: k.type for k in r.get_all_topics_and_types()}; names = ["/scan", "/odom"]; r.set_filter(rosbag2_py.StorageFilter(topics=names)); M = {k: get_message(ty[k]) for k in names}
    scans, od = [], []
    while r.has_next():
        tp, raw, tn = r.read_next(); m = deserialize_message(raw, M[tp]); t = tn / 1e9
        if tp == "/scan": scans.append((t, np.array(m.ranges, dtype=float)))
        else: od.append((t, m.twist.twist.linear.x, m.twist.twist.angular.z))
    od = np.array(od); still = (np.abs(od[:, 1]) < 0.005) & (np.abs(od[:, 2]) < 0.01); ts = np.array([s[0] for s in scans]); F = np.array([s[1] for s in scans])
    st = np.interp(ts, od[:, 0], still.astype(float)) > 0.999; i = 0; n_str = 0
    while i < len(ts):
        if st[i]:
            j = i
            while j + 1 < len(ts) and st[j + 1]: j += 1
            if ts[j] - ts[i] >= 4.0 and j - i >= 35:
                X = F[i:j + 1]; v = np.isfinite(X) & (X > 0.05); p = v.mean(0); ok = p >= 0.5
                if ok.sum() > 50:
                    Xm = np.where(v, X, np.nan); med = np.nanmedian(Xm[:, ok], 0); dev = Xm[:, ok] - med
                    core = np.where(np.abs(dev) < 0.10, dev, np.nan); core_sd = np.nanstd(core, 0)
                    outl = np.isfinite(dev) & (np.abs(dev) >= 0.10); q = outl.sum(0) / np.maximum(np.isfinite(dev).sum(0), 1)
                    nearer = ((dev < 0) & outl).sum(0) / np.maximum(outl.sum(0), 1); mag = np.where(outl.any(0), np.nanmedian(np.where(outl, np.abs(dev), np.nan), 0), np.nan)
                    for a, b, c, d, e, f in zip(med, core_sd, q, nearer, mag, 1 - p[ok]): rows.append((a, b, c, d, e, f))
                n_str += 1
            i = j + 1
        else: i += 1
    print(f"{os.path.basename(os.path.dirname(bag)) or bag}: {len(ts)} scans, {n_str} stationary stretches")
R = np.array(rows); out = {}
print(f"\n{len(R)} beam-stretches (beams with a return in >= 50 % of the frames). Per range bin of the beam's median range:")
print("range (m)    | beams | core sigma (|dev|<10 cm) median / p90 (mm) | outlier rate (|dev|>=10 cm) mean / p90 | outliers nearer | outlier |dev| median / p90 (cm) | dropout mean / p90")
for lo, hi in zip(BINS[:-1], BINS[1:]):
    s_ = R[(R[:, 0] >= lo) & (R[:, 0] < hi)]
    if len(s_) < 30: continue
    has = np.isfinite(s_[:, 4]); m = s_[has]
    print(f"{lo:5.1f}-{hi:5.1f} | {len(s_):5d} | {np.nanmedian(s_[:,1])*1000:6.1f} / {np.nanpercentile(s_[:,1],90)*1000:6.1f} | {100*np.mean(s_[:,2]):5.1f} % / {100*np.percentile(s_[:,2],90):5.1f} % | {100*np.mean(m[:,3]):4.0f} % | {np.median(m[:,4])*100:5.1f} / {np.percentile(m[:,4],90)*100:5.1f} | {100*np.mean(s_[:,5]):5.1f} % / {100*np.percentile(s_[:,5],90):5.1f} %")
    out[f"{lo}-{hi}"] = dict(n=int(len(s_)), core_sigma_med_mm=float(np.nanmedian(s_[:, 1]) * 1000), core_sigma_p90_mm=float(np.nanpercentile(s_[:, 1], 90) * 1000), outlier_rate_mean=float(np.mean(s_[:, 2])), outlier_rate_p90=float(np.percentile(s_[:, 2], 90)),
                             outlier_nearer=float(np.mean(m[:, 3])), outlier_dev_med_cm=float(np.median(m[:, 4]) * 100), outlier_dev_p90_cm=float(np.percentile(m[:, 4], 90) * 100), dropout_mean=float(np.mean(s_[:, 5])), dropout_p90=float(np.percentile(s_[:, 5], 90)))
json.dump(out, open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "lidar_noise.json"), "w"), indent=1)
