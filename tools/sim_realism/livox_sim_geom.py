"""Geometry check of the simulated Mid-360 against Gazebo's own gpu_lidar (625 x 50 grid, same mount) in the running simulator. Runs inside the sim image (numpy, scipy, rosbag2_py).
usage: livox_sim_geom.py BAG_STATIC BAG_ROT       bags hold /livox_mid360/points_raw (Gazebo), /livox/lidar (ours, CustomMsg), /odom_gt_raw
static: every valid point of ours (30 frames) vs the Gazebo ray with the nearest direction (< 0.35 deg): range difference.
rotating: both clouds into the world frame with the ground-truth pose, ours per point time; distance to the nearest Gazebo point (all Gazebo frames of the run). Controls: ours with the frame-start pose for all points (no distortion handling)."""
import sys

import numpy as np
from scipy.spatial import cKDTree

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

SENSOR_Z = 0.28
PC2 = {1: "i1", 2: "u1", 3: "<i2", 4: "<u2", 5: "<i4", 6: "<u4", 7: "<f4", 8: "<f8"}


def read(bag, topics):
    r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
    ty = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
    while r.has_next():
        n, raw, _ = r.read_next()
        if n in topics: yield n, deserialize_message(raw, ty[n])


def stamp(h): return h.stamp.sec + h.stamp.nanosec * 1e-9


def load(bag):
    gz, ours, odom = [], [], []
    for n, m in read(bag, ("/livox_mid360/points_raw", "/livox/lidar", "/odom_gt_raw")):
        if n == "/odom_gt_raw":
            q = m.pose.pose.orientation; odom.append((stamp(m.header), m.pose.pose.position.x, m.pose.pose.position.y, np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))))
        elif n == "/livox/lidar":
            p = m.points; xyz = np.array([(q.x, q.y, q.z) for q in p], np.float32); t = np.array([q.offset_time for q in p], np.float64) * 1e-9 + m.timebase * 1e-9
            ours.append((stamp(m.header), xyz, t))
        else:
            dt = np.dtype({"names": [f.name for f in m.fields], "formats": [PC2[f.datatype] for f in m.fields], "offsets": [f.offset for f in m.fields], "itemsize": m.point_step})
            a = np.frombuffer(bytes(m.data), dt, m.width * m.height); xyz = np.column_stack([a["x"], a["y"], a["z"]]).astype(np.float32); gz.append((stamp(m.header), xyz[np.isfinite(xyz).all(1)]))
    return gz, ours, np.array(sorted(odom))


def pose_at(od, t):
    yaw = np.unwrap(od[:, 3])
    return np.interp(t, od[:, 0], od[:, 1]), np.interp(t, od[:, 0], od[:, 2]), np.interp(t, od[:, 0], yaw)


def to_world(xyz, x, y, yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    return np.column_stack([x + c * xyz[:, 0] - s * xyz[:, 1], y + s * xyz[:, 0] + c * xyz[:, 1], SENSOR_Z + xyz[:, 2]])


def dirs(xyz):
    r = np.linalg.norm(xyz, axis=1); return xyz / r[:, None], r


def pct(a, q=(50, 90, 99)): return " ".join(f"p{p} {np.percentile(a, p):.3f}" for p in q)


# ---- static
gz, ours, od = load(sys.argv[1])
print(f"static: {len(gz)} Gazebo frames, {len(ours)} ours, {len(od)} odom; first pose x {od[0,1]:.2f} y {od[0,2]:.2f} yaw {od[0,3]:.3f}")
gd, gr = dirs(gz[len(gz) // 2][1]); tree = cKDTree(gd); D, R, DR = [], [], []
for st, xyz, t in ours[10:40]:
    v = np.linalg.norm(xyz, axis=1) > 0; d, r = dirs(xyz[v]); dist, ix = tree.query(d); ang = np.degrees(2 * np.arcsin(np.clip(dist / 2, 0, 1))); ok = ang < 0.35
    D.append(ang[ok]); R.append(r[ok]); DR.append(r[ok] - gr[ix[ok]])
    nomatch = (~ok).sum()
D, R, DR = map(np.concatenate, (D, R, DR)); a = np.abs(DR)
print(f"  {len(DR)} points matched (direction < 0.35 deg, mean {D.mean():.2f}); range difference ours - Gazebo: median {np.median(DR):+.3f} m, |d| {pct(a)}; share |d| < 0.05 m: {100*(a<0.05).mean():.1f} %, < 0.20 m: {100*(a<0.20).mean():.1f} %")
for lo, hi in ((0, 3), (3, 6), (6, 12)):
    m = (R >= lo) & (R < hi)
    if m.sum(): print(f"    range {lo}-{hi} m: n {m.sum()}, median {np.median(DR[m]):+.3f}, |d| {pct(a[m])}")

# ---- rotating
gz, ours, od = load(sys.argv[2]); print(f"rotating: {len(gz)} Gazebo frames, {len(ours)} ours; yaw span {np.degrees(np.ptp(np.unwrap(od[:,3]))):.0f} deg over {od[-1,0]-od[0,0]:.1f} s")
G = np.concatenate([to_world(x, *pose_at(od, st)) for st, x in gz]); gt = cKDTree(G)
res = {"per-point pose (ours)": [], "frame-start pose (no distortion handling)": []}
for st, xyz, t in ours[10:-3]:
    v = np.linalg.norm(xyz, axis=1) > 0; px, py, pyaw = pose_at(od, t[v])
    res["per-point pose (ours)"].append(gt.query(to_world(xyz[v], px, py, pyaw))[0])
    sx, sy, syaw = pose_at(od, st); res["frame-start pose (no distortion handling)"].append(gt.query(to_world(xyz[v], sx, sy, syaw))[0])
for k, v in res.items():
    v = np.concatenate(v); print(f"  {k:44s} n {len(v)}: distance to nearest Gazebo point (m) {pct(v)}; share < 0.05 m {100*(v<0.05).mean():.1f} %")
