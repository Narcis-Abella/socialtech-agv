"""Where do /scan endpoints land on the map? Independent check of the z cut, sensor height and frames: puts N scans of a running bench at the
REFERENCE pose (not AMCL's) and prints the share of endpoints on or next to an occupied PGM cell (1 px dilation). Run while the bag plays.
usage: scan_vs_map.py <alidarState.txt> <map.yaml> [--n 30] [--ref-dt 0]   (the /scan frame is base_footprint: x, y of the body, yaw of the body)"""
import argparse

import numpy as np

import pose_eval as pe


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("ref")
    ap.add_argument("map")
    ap.add_argument("--n", type=int, default=30)
    ap.add_argument("--ref-dt", type=float, default=0.0)
    a = ap.parse_args()
    m = pe.read_map_yaml(a.map)
    grid = pe.read_pgm(m["image"])
    occ = grid < 50
    near = occ.copy()
    for dr in (-1, 0, 1):
        for dc in (-1, 0, 1):
            near |= np.roll(np.roll(occ, dr, 0), dc, 1)
    probe = np.where(near, 0, 254).astype(np.uint8)
    t_ref, ref = pe.reference_map(a.ref, m["T_map_world"], a.ref_dt)

    import rclpy
    from sensor_msgs.msg import LaserScan

    rclpy.init()
    node = rclpy.create_node("scan_vs_map")
    shares = []

    def cb(s):
        r = np.asarray(s.ranges)
        ok = np.isfinite(r) & (r > s.range_min) & (r < s.range_max)
        ang = s.angle_min + np.arange(len(r)) * s.angle_increment
        x, y, yaw = pe.interp(t_ref, ref, np.array([s.header.stamp.sec + s.header.stamp.nanosec * 1e-9]))[0]
        if np.isnan(x) or not ok.any():
            return
        c, sn = np.cos(yaw), np.sin(yaw)
        px, py = r[ok] * np.cos(ang[ok]), r[ok] * np.sin(ang[ok])
        pts = np.c_[x + c * px - sn * py, y + sn * px + c * py]
        shares.append((pe.cells(probe, m["resolution"], m["origin"], pts) < 50).mean() * 100)
        print(f"scan {len(shares)}: {ok.sum()} endpoints, {shares[-1]:.0f} % on/next to occupied cells", flush=True)
        if len(shares) >= a.n:
            print(f"MEAN {np.mean(shares):.1f} % over {len(shares)} scans")
            rclpy.shutdown()

    node.create_subscription(LaserScan, "/scan", cb, 10)
    rclpy.spin(node)


if __name__ == "__main__":
    main()
