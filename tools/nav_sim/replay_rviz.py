"""Replays recorded trials in rviz2 on the laptop (no simulator, no board): map, planned path, driven path, an animated footprint, and at each trial's
minimum mid-route clearance the footprint, the nearest wall cell and two distances: the metric analyze.py reports (footprint to occupied-cell CENTRE, 5 cm grid)
and an independent one computed here (footprint to the wall FACE of the nearest cells).
  replay_rviz.py --map maps/loyola_gt.yaml --case base:<run>/out:p12 --case infl70:<run>/out:p12 [--speed 4] [--no-rviz]
Cases of one call share the map, so use the same world and trial id to compare variants."""
import argparse
import math
import os
import subprocess
import sys
import tempfile
import time

import numpy as np
from scipy.spatial import cKDTree

import analyze as a
import metrics as m
import scenarios as sc

COLORS = [(0.9, 0.2, 0.2), (0.2, 0.7, 0.3), (0.2, 0.4, 0.95), (0.9, 0.6, 0.1)]
BY_LABEL = {"base": COLORS[0], "infl70": COLORS[1], "la5": COLORS[2]}      # a variant keeps its colour in every window; other labels take the next free one
STEP = 0.01     # footprint sampling for the independent distance


def face_clearance(occ, origin, res, x, y, yaw, length, width):
    """(gap, (row, col) of the nearest occupied cell, footprint point nearest to it): minimum distance from the footprint rectangle to the SQUARE of any occupied cell; 0 if it covers one; (inf, None, None) without obstacles."""
    h = occ.shape[0]
    u, v = np.meshgrid(np.arange(-length / 2, length / 2 + 1e-9, STEP), np.arange(-width / 2, width / 2 + 1e-9, STEP))
    c, s = math.cos(yaw), math.sin(yaw)
    px, py = (x + c * u - s * v).ravel(), (y + s * u + c * v).ravel()
    reach = 0.5 * math.hypot(length, width) + 1.5
    rows, cols = np.nonzero(occ)
    cx, cy = origin[0] + (cols + 0.5) * res, origin[1] + (h - 1 - rows + 0.5) * res
    near = np.hypot(cx - x, cy - y) < reach
    if not near.any():
        return math.inf, None, None
    rows, cols, cx, cy = rows[near], cols[near], cx[near], cy[near]
    k = min(4, len(cx))
    _, idx = cKDTree(np.c_[cx, cy]).query(np.c_[px, py], k=k)
    idx = idx.reshape(len(px), -1)
    dx = np.maximum(np.abs(px[:, None] - cx[idx]) - res / 2, 0.0)
    dy = np.maximum(np.abs(py[:, None] - cy[idx]) - res / 2, 0.0)
    d = np.hypot(dx, dy)
    i, j = np.unravel_index(np.argmin(d), d.shape)
    return float(d[i, j]), (int(rows[idx[i, j]]), int(cols[idx[i, j]])), (float(px[i]), float(py[i]))


def load_case(spec, ctx, occ):
    label, run, tid = spec.split(":")
    rec = next(r for r in a.load_trials(run) if r["id"] == tid)
    col = {c: i for i, c in enumerate(rec["cols"])}
    rows = np.asarray(rec["rows"], float)
    t, pose = rows[:, col["t"]] - rows[0, col["t"]], rows[:, [col["gt_x"], col["gt_y"], col["gt_yaw"]]]
    sub = pose[::5]                         # same sampling as analyze.trial_metrics
    cl = np.array([m.footprint_clearance(ctx["dist"], ctx["origin"], ctx["res"], *p, *ctx["footprint"]) for p in sub])
    away = (np.hypot(*(sub[:, :2] - rec["start"][:2]).T) > 1.0) & (np.hypot(*(sub[:, :2] - rec["goal"][:2]).T) > 1.0)
    i = int(np.flatnonzero(away)[np.argmin(cl[away])]) if away.any() else int(np.argmin(cl))
    gap, cell, near = face_clearance(occ, ctx["origin"], ctx["res"], *sub[i], *ctx["footprint"])
    return {"label": label, "id": tid, "t": t, "pose": pose, "plan": np.asarray(rec["plan_first"], float), "min_pose": sub[i], "metric": float(cl[i]), "face": gap, "cell": cell,
            "near": near, "duration": float(rec["duration"]), "status": rec["status"]}


def colours(labels):
    free = [c for c in COLORS if c not in BY_LABEL.values()] + COLORS
    return [BY_LABEL.get(l) or free.pop(0) for l in labels]


def rect(x, y, yaw, length, width):
    c, s = math.cos(yaw), math.sin(yaw)
    pts = [(length / 2, width / 2), (-length / 2, width / 2), (-length / 2, -width / 2), (length / 2, -width / 2), (length / 2, width / 2)]
    return [(x + c * u - s * v, y + s * u + c * v) for u, v in pts]


def rviz_config(path, x, y, scale):
    open(path, "w").write(f"""Visualization Manager:
  Class: ""
  Displays:
    - Class: rviz_default_plugins/Map
      Name: Map
      Enabled: true
      Topic: {{Value: /map, Durability Policy: Transient Local, Reliability Policy: Reliable}}
      Color Scheme: map
    - Class: rviz_default_plugins/MarkerArray
      Name: Replay
      Enabled: true
      Topic: {{Value: /replay/markers}}
  Global Options: {{Fixed Frame: map, Background Color: 40; 40; 40}}
  Name: root
  Tools: []
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/TopDownOrtho
      Name: Current View
      Scale: {scale}
      X: {x}
      Y: {y}
      Target Frame: <Fixed Frame>
      Value: TopDownOrtho (rviz_default_plugins)
Window Geometry: {{Height: 1000, Width: 1500}}
""")


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--map", required=True)
    ap.add_argument("--case", action="append", required=True, help="label:run out dir:trial id")
    ap.add_argument("--speed", type=float, default=4.0)
    ap.add_argument("--no-rviz", action="store_true", help="publish only (rviz2 started elsewhere, or a test)")
    args = ap.parse_args(argv)
    mp = sc.load_map(args.map)
    ctx = a.context(args.map)
    cases = [load_case(c, ctx, mp["occ"]) for c in args.case]
    for c in cases:
        print(f"{c['label']:10s} {c['id']} {c['status']} {c['duration']:.1f} s  min mid-route clearance: metric {c['metric'] * 100:.1f} cm (to cell centre) | independent {c['face'] * 100:.1f} cm (to wall face)")
    import rclpy
    from geometry_msgs.msg import Point
    from nav_msgs.msg import OccupancyGrid
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from visualization_msgs.msg import Marker, MarkerArray
    rclpy.init()
    node = rclpy.create_node("replay_rviz")
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    map_pub, mk_pub = node.create_publisher(OccupancyGrid, "/map", latched), node.create_publisher(MarkerArray, "/replay/markers", 10)
    h, w = mp["occ"].shape
    grid = np.where(mp["occ"], 100, np.where(mp["blocked"], -1, 0)).astype(np.int8)[::-1]      # image row 0 is the top; OccupancyGrid row 0 is y min
    og = OccupancyGrid()
    og.header.frame_id, og.info.resolution, og.info.width, og.info.height = "map", mp["res"], w, h
    og.info.origin.position.x, og.info.origin.position.y, og.info.origin.orientation.w = mp["origin"][0], mp["origin"][1], 1.0
    og.data = grid.ravel().tolist()
    map_pub.publish(og)
    fp = ctx["footprint"]
    allp = np.vstack([c["pose"][:, :2] for c in cases])
    centre, span = allp.mean(axis=0), np.ptp(allp, axis=0).max() + 4.0
    scale = 900.0 / span
    rcfg = os.path.join(tempfile.mkdtemp(prefix="replay_"), "replay.rviz")
    rviz_config(rcfg, float(centre[0]), float(centre[1]), scale)
    proc = None if args.no_rviz else subprocess.Popen(["rviz2", "-d", rcfg], env={**os.environ, "QT_QPA_PLATFORM": "xcb"})

    def line(mid, ns, pts, rgb, width, alpha=1.0):
        k = Marker()
        k.header.frame_id, k.ns, k.id, k.type, k.action = "map", ns, mid, Marker.LINE_STRIP, Marker.ADD
        k.scale.x, k.color.r, k.color.g, k.color.b, k.color.a = width, *rgb, alpha
        k.points = [Point(x=float(x), y=float(y), z=0.05) for x, y in pts]
        return k

    def text(mid, ns, x, y, s, rgb, size=0.35):
        k = Marker()
        k.header.frame_id, k.ns, k.id, k.type, k.action = "map", ns, mid, Marker.TEXT_VIEW_FACING, Marker.ADD
        k.pose.position.x, k.pose.position.y, k.pose.position.z = float(x), float(y), 0.3
        k.scale.z, k.color.r, k.color.g, k.color.b, k.color.a, k.text = size, *rgb, 1.0, s
        return k

    static = []
    for n, (c, rgb) in enumerate(zip(cases, colours([c["label"] for c in cases]))):
        static.append(line(10 * n, "plan", c["plan"][:, :2], rgb, 0.02, 0.6))
        static.append(line(10 * n + 1, "driven", c["pose"][:, :2], rgb, 0.05))
        mx, my, myaw = c["min_pose"]
        static.append(line(10 * n + 2, "minfp", rect(mx, my, myaw, *fp), rgb, 0.04))
        static.append(text(10 * n + 3, "label", mx, my + 0.6 + 0.35 * n, f"{c['label']}: {c['metric'] * 100:.1f} cm (centre) / {c['face'] * 100:.1f} cm (face)", rgb))
        if c["cell"] is not None:
            r, cc = c["cell"]
            wx, wy = mp["origin"][0] + (cc + 0.5) * mp["res"], mp["origin"][1] + (h - 1 - r + 0.5) * mp["res"]
            static.append(line(10 * n + 4, "gap", [c["near"], (wx, wy)], (1.0, 1.0, 0.0), 0.02))
            k = Marker()
            k.header.frame_id, k.ns, k.id, k.type, k.action = "map", "wallcell", 10 * n + 5, Marker.CUBE, Marker.ADD
            k.pose.position.x, k.pose.position.y, k.pose.position.z, k.pose.orientation.w = wx, wy, 0.02, 1.0
            k.scale.x = k.scale.y = float(mp["res"])
            k.scale.z, k.color.r, k.color.g, k.color.b, k.color.a = 0.04, 1.0, 1.0, 0.0, 0.9
            static.append(k)
    pause, t_wall = 2.0, time.time()
    longest = max(c["t"][-1] for c in cases)
    try:
        while rclpy.ok() and (proc is None or proc.poll() is None):
            ph = ((time.time() - t_wall) * args.speed) % (longest + pause * args.speed)
            msgs = list(static)
            for n, (c, rgb) in enumerate(zip(cases, colours([c["label"] for c in cases]))):
                tt = min(ph, c["t"][-1])
                x, y = np.interp(tt, c["t"], c["pose"][:, 0]), np.interp(tt, c["t"], c["pose"][:, 1])
                yaw = float(np.interp(tt, c["t"], np.unwrap(c["pose"][:, 2])))
                msgs.append(line(10 * n + 6, "robot", rect(x, y, yaw, *fp), rgb, 0.03))
            mk_pub.publish(MarkerArray(markers=msgs))
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        if proc is not None and proc.poll() is None:
            proc.terminate()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv[1:])
