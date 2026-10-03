"""IMU-vs-floor offset of a Livox sensor from raw bags (no SLAM involved).

While the robot stands still, the IMU says where "up" is (mean acceleration direction u) and the LiDAR sees the floor
and the ceiling as planes. On a level floor their normals are the true up direction, so
  offset = (plane normal) - u        (sensor frame, small vector; x/y components in degrees)
is the accelerometer bias or IMU-LiDAR misalignment that tilts GLIM's map (GLIM takes "up" from the IMU at start).
If it is a property of the sensor it is the same vector in every bag, whatever the robot heading or place.

  static_offset.py extract <bag_dir> <out.json> [lidar_topic] [imu_topic] [acc_std]   (needs ROS 2: rosbag2_py; run in the robot image)
  static_offset.py report <out.json>...                                      (numpy only: per-bag offsets + leave-one-bag-out)
  static_offset.py overlay <bag_to_exclude|-> <out.json>...                  (prints a glim_eval overlay: T_lidar_imu rotated by the pooled CEILING offset)

Floor/ceiling planes: floor_tilt.fit_plane on points rotated so that u is z; floor = points below the sensor, ceiling = above.
"""
import json
import sys
from pathlib import Path

import numpy as np

import floor_tilt

WIN = 2.0          # s of robot standing still needed for a window
MIN_SPAN = 2.0     # m, horizontal extent a plane must cover (a tabletop is not a floor)
EDGE = 0.1         # s trimmed at each end of a window when picking LiDAR scans (IMU and LiDAR clocks differ slightly)
DTYPE = {1: "i1", 2: "u1", 3: "i2", 4: "u2", 5: "i4", 6: "u4", 7: "f4", 8: "f8"}


def static_windows(t, acc, gyr, win=WIN, min_gap=10.0, max_n=12, acc_std=0.004):
    """Index pairs (i0, i1) of non-overlapping windows where gyro and accelerometer std are tiny (the gyro bias is ~0.02 rad/s, so judge the std, not the level)."""
    n = int(win / np.median(np.diff(t)))
    out, last, i = [], -1e9, 0
    while i < len(t) - n and len(out) < max_n:  # slide by a quarter window: stops rarely line up with a fixed grid
        if t[i] - last >= min_gap and gyr[i:i + n].std(0).max() < 0.004 and acc[i:i + n].std(0).max() < acc_std:
            out.append((i, i + n))
            last = t[i]
            i += n
        else:
            i += max(1, n // 4)
    return out


def rot_to_z(u):
    """Rotation matrix R with R @ u = z (Rodrigues)."""
    z = np.array([0.0, 0.0, 1.0])
    v = np.cross(u, z)
    s, c = np.linalg.norm(v), float(u @ z)
    if s < 1e-12:
        return np.eye(3)
    K = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + K + K @ K * (1 - c) / s**2


def level_planes(P, u, rng):
    """Floor and ceiling of a point set (sensor frame) as {"n" (up-facing unit normal), "h" (distance to the sensor), "inliers"} or None."""
    R = rot_to_z(u)
    Q = P @ R.T
    out = {}
    for kind, sel in (("floor", Q[:, 2] < 0), ("ceiling", Q[:, 2] > 0)):
        q = Q[sel]
        plane = floor_tilt.fit_plane(q, rng) if len(q) >= 3 else None
        if plane is not None:
            v, d, n = plane
            inl = q[np.abs(q @ v - d) < floor_tilt.TOL]
            if max(np.ptp(inl[:, 0]), np.ptp(inl[:, 1])) < MIN_SPAN:
                plane = None
        out[kind] = None if plane is None else {"n": R.T @ v, "h": abs(d), "inliers": n}
    return out


def cloud_xyz(msg):
    """PointCloud2 -> (N, 3) float64 without NaN and the sensor's own zeros."""
    dt = np.dtype({"names": [f.name for f in msg.fields], "formats": [DTYPE[f.datatype] for f in msg.fields],
                   "offsets": [f.offset for f in msg.fields], "itemsize": msg.point_step})
    a = np.frombuffer(bytes(msg.data), dtype=dt, count=msg.width * msg.height)
    xyz = np.column_stack([a["x"], a["y"], a["z"]]).astype(np.float64)
    return xyz[np.isfinite(xyz).all(1) & (np.linalg.norm(xyz, axis=1) > 0.3)]


def extract(bag, out, lidar_topic="/livox/lidar", imu_topic="/livox/imu", acc_std=0.004):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Imu, PointCloud2

    storage = next((l.split(":")[1].strip() for l in (Path(bag) / "metadata.yaml").read_text().splitlines() if "storage_identifier" in l), "mcap")

    def reader(topic):
        r = rosbag2_py.SequentialReader()
        r.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=storage), rosbag2_py.ConverterOptions("", ""))
        r.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
        return r

    ts, rows = [], []
    r = reader(imu_topic)
    while r.has_next():
        _, data, t_ns = r.read_next()
        m = deserialize_message(data, Imu)
        ts.append(t_ns * 1e-9)  # bag time: lets the LiDAR pass pick scans without decoding the others
        rows.append([m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z, m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z])
    t, d = np.array(ts), np.array(rows)
    wins = static_windows(t, d[:, :3], d[:, 3:], acc_std=float(acc_std))  # a robot standing with its motors on vibrates (acc std ~0.005 g): zero-mean, so 0.007 is fine there
    span = [(t[i] + EDGE, t[j - 1] - EDGE) for i, j in wins]
    clouds = [[] for _ in wins]
    r = reader(lidar_topic)
    while r.has_next():
        _, data, t_ns = r.read_next()
        for k, (a, b) in enumerate(span):
            if a <= t_ns * 1e-9 <= b:
                clouds[k].append(cloud_xyz(deserialize_message(data, PointCloud2)))
    rng = np.random.default_rng(0)
    result = []
    for (i, j), cl in zip(wins, clouds):
        if not cl:
            continue
        u = d[i:j, :3].mean(0)
        u /= np.linalg.norm(u)
        planes = level_planes(np.vstack(cl), u, rng)
        result.append({"t": float(t[i] - t[0]), "scans": len(cl), "u": u.tolist(),
                       **{k: None if v is None else {"n": v["n"].tolist(), "h": v["h"], "inliers": v["inliers"]} for k, v in planes.items()}})
    Path(out).write_text(json.dumps({"bag": Path(bag).name, "windows": result}, indent=1))
    print(f"{out}: {len(result)} static windows, {sum(w['floor'] is not None for w in result)} with a floor, {sum(w['ceiling'] is not None for w in result)} with a ceiling")


def summary(bags):
    """bags: {name: [offset x/y (rad), ...]}. Median per bag, and the leave-one-bag-out residual: how far the bag's offset is from the median of the OTHER bags' medians (what a correction learned elsewhere would leave)."""
    med = {k: np.median(np.array(v), axis=0) for k, v in bags.items() if v}
    out = {}
    for k, m in med.items():
        others = [v for kk, v in med.items() if kk != k]
        ref = np.median(others, axis=0) if others else None
        out[k] = {"n": len(bags[k]), "median": m, "spread_deg": float(np.degrees(np.linalg.norm(np.array(bags[k]).std(0)))),
                  "size_deg": float(np.degrees(np.linalg.norm(m))),
                  "loo_residual_deg": None if ref is None else float(np.degrees(np.linalg.norm(m - ref)))}
    return out


XYZ = [0.011, 0.02329, -0.04412]  # T_lidar_imu translation, Mid-360 manual (tools/glim_eval/base.json)


def correction_quaternion(off):
    """Quaternion (x, y, z, w) of the rotation R with R @ u_imu = u_imu + (off_x, off_y, 0) for u_imu ~ z: rotation vector (-off_y, off_x, 0)."""
    w = np.array([-off[1], off[0], 0.0])
    q = np.array([*(w / 2), 1.0])
    return q / np.linalg.norm(q)


def pooled_offset(data, exclude=None, kind="ceiling"):
    """Median over bags of each bag's median offset (x, y in rad). The floor from raw single windows is too noisy (~1 deg per window); the ceiling is not."""
    med = [np.median([(np.array(w[kind]["n"]) - np.array(w["u"]))[:2] for w in ws if w[kind]], axis=0)
           for b, ws in data.items() if b != exclude and any(w[kind] for w in ws)]
    return np.median(med, axis=0)


def load(files):
    return {j["bag"]: j["windows"] for j in (json.loads(Path(f).read_text()) for f in files)}


def overlay(exclude, files):
    off = pooled_offset(load(files), None if exclude == "-" else exclude)
    q = correction_quaternion(off)
    return {"config_sensors.json": {"sensors": {"T_lidar_imu": XYZ + [round(float(v), 6) for v in q]}}}


def report(files):
    data = load(files)
    for kind in ("floor", "ceiling"):
        bags = {b: [(np.array(w[kind]["n"]) - np.array(w["u"]))[:2] for w in ws if w[kind]] for b, ws in data.items()}
        print(f"\n### offset from the {kind} (plane normal - IMU up; sensor frame)\n| Bag | Windows | Offset x deg | Offset y deg | Size deg | Spread deg | Leave-one-bag-out residual deg |\n|---|---|---|---|---|---|---|")
        for b, s in summary(bags).items():
            m = np.degrees(s["median"])
            print(f"| {b} | {s['n']} | {m[0]:+.3f} | {m[1]:+.3f} | {s['size_deg']:.3f} | {s['spread_deg']:.3f} | {'-' if s['loo_residual_deg'] is None else format(s['loo_residual_deg'], '.3f')} |")
        pooled = np.median([np.median(v, axis=0) for v in bags.values() if v], axis=0)
        print(f"median over bags: x {np.degrees(pooled[0]):+.3f} deg, y {np.degrees(pooled[1]):+.3f} deg (sensor-frame rotation vector = (-y, x, 0))")


if __name__ == "__main__":
    if len(sys.argv) >= 4 and sys.argv[1] == "extract":
        extract(*sys.argv[2:])
    elif len(sys.argv) >= 3 and sys.argv[1] == "report":
        report(sys.argv[2:])
    elif len(sys.argv) >= 4 and sys.argv[1] == "overlay":
        print(json.dumps(overlay(sys.argv[2], sys.argv[3:]), indent=2))
    else:
        sys.exit(__doc__)
