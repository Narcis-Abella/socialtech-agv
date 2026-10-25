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
from scipy import ndimage
from scipy.spatial import cKDTree

import analyze as a
import dyn
import dyn_metrics as dm
import gcm_codec
import ghost
import metrics as m
import scenarios as sc

COLORS = [(0.9, 0.2, 0.2), (0.2, 0.7, 0.3), (0.2, 0.4, 0.95), (1.0, 0.0, 0.0), (0.0, 0.3, 1.0), (1.0, 0.6, 0.0)]
BY_LABEL = {"base": COLORS[0], "infl70": COLORS[1], "la5": COLORS[2], "cd08m": COLORS[3], "vm": COLORS[4], "cml": COLORS[4], "cml2": COLORS[5], "actual": COLORS[1], "nueva": COLORS[3]}      # a variant keeps its colour in every window; other labels take the next free one
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


def walker_series(rec, t0):
    """(T, X, Y, YAW, size) of the first moving obstacle of a trial with "dyn_log", times relative to t0; None without one."""
    log = [e for e in rec.get("dyn_log", []) if e[1] == (rec["dyn"][0]["model"] if rec.get("dyn") else None)]
    if not log:
        return None
    arr = np.asarray([[e[0] - t0, e[2], e[3], e[4]] for e in log], float)
    return arr[:, 0], arr[:, 1], arr[:, 2], arr[:, 3], tuple(rec["dyn"][0]["size"][:2])


def walker_at(w, t):
    """Walker pose (x, y, yaw) at time t from the last command sent at or before t; None before it starts moving."""
    i = int(np.searchsorted(w[0], t, side="right")) - 1
    return None if i < 0 else (w[1][i], w[2][i], w[3][i])


def load_case(spec, ctx, occ):
    label, run, tid = spec.split(":")
    rec = next(r for r in a.load_trials(run) if r["id"] == tid)
    col = {c: i for i, c in enumerate(rec["cols"])}
    rows = np.asarray(rec["rows"], float)
    t, pose = rows[:, col["t"]] - rows[0, col["t"]], rows[:, [col["gt_x"], col["gt_y"], col["gt_yaw"]]]
    w = walker_series(rec, rows[0, col["t"]])
    extra = {"gcm": [[tf - rows[0, col["t"]], pl] for tf, pl in rec.get("gcm_frames", [])], "gcm_meta": rec.get("gcm_meta"), "cm": [[tc - rows[0, col["t"]], ac, pc] for tc, ac, pc in rec.get("cm_log", [])]}      # recorded global costmap (REC_GCM=1), times relative to the trial start
    if w is not None:                       # encounter with a moving obstacle: report the closest approach to it instead of the wall clearance
        gaps = []
        for tt, p in zip(t, pose):
            o = walker_at(w, tt)
            gaps.append((dyn.rect_distance((*p, *dm.BODY), (*o, *w[4])), dyn.rect_distance((*p, *dm.FOOTPRINT), (*o, *w[4]))) if o and math.hypot(o[0] - p[0], o[1] - p[1]) < dm.NEAR else (dm.NEAR, dm.NEAR))
        gaps = np.asarray(gaps)
        k = int(np.argmin(gaps[:, 0]))
        d = dm.trial_metrics(rec)
        return {"label": label, "id": tid, "t": t, "pose": pose, "plan": np.asarray(rec["plan_first"], float), "min_pose": pose[k], "metric": float(gaps[k, 0]), "face": float(gaps[k, 1]), "cell": None,
                "near": None, "duration": float(rec["duration"]), "status": rec["status"], "walker": w, "dyn": d, "start": rec["start"], "goal": rec["goal"], **extra}
    sub = pose[::5]                         # same sampling as analyze.trial_metrics
    cl = np.array([m.footprint_clearance(ctx["dist"], ctx["origin"], ctx["res"], *p, *ctx["footprint"]) for p in sub])
    away = (np.hypot(*(sub[:, :2] - rec["start"][:2]).T) > 1.0) & (np.hypot(*(sub[:, :2] - rec["goal"][:2]).T) > 1.0)
    i = int(np.flatnonzero(away)[np.argmin(cl[away])]) if away.any() else int(np.argmin(cl))
    gap, cell, near = face_clearance(occ, ctx["origin"], ctx["res"], *sub[i], *ctx["footprint"])
    return {"label": label, "id": tid, "t": t, "pose": pose, "plan": np.asarray(rec["plan_first"], float), "min_pose": sub[i], "metric": float(cl[i]), "face": gap, "cell": cell,
            "near": near, "duration": float(rec["duration"]), "status": rec["status"], "start": rec["start"], "goal": rec["goal"], **extra}


def colours(labels):
    free = [c for c in COLORS if c not in BY_LABEL.values()] + COLORS
    seen, out = {}, []
    for l in labels:          # one colour per distinct label, however many cases share it
        if l not in seen:
            seen[l] = BY_LABEL.get(l) or free.pop(0)
        out.append(seen[l])
    return out


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
    - Class: rviz_default_plugins/Map
      Name: Global costmap (recorded)
      Enabled: true
      Alpha: 0.6
      Color Scheme: costmap
      Topic: {{Value: /replay/global_costmap, Reliability Policy: Reliable}}
    - Class: rviz_default_plugins/MarkerArray
      Name: Unexplained lethal cells (my count, switch off to judge by eye)
      Enabled: true
      Topic: {{Value: /replay/wake}}
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
    ap.add_argument("--sequence", action="store_true", help="play the cases one after another (each with its own recorded costmap) instead of overlaying them")
    ap.add_argument("--zone", action="append", help="front,back,half_width[,polygon name] of a Collision Monitor zone to draw on the robot (metres); repeat for several. Red when that polygon is in command (cm_log), yellow when the approach is, grey otherwise")
    ap.add_argument("--no-rviz", action="store_true", help="publish only (rviz2 started elsewhere, or a test)")
    args = ap.parse_args(argv)
    mp = sc.load_map(args.map)
    ctx = a.context(args.map)
    cases = [load_case(c, ctx, mp["occ"]) for c in args.case]
    for c in cases:
        if c.get("walker"):
            d = c["dyn"]
            print(f"{c['label']:10s} {c['id']} {c['status']} {c['duration']:.1f} s  closest approach to the walker: body {c['metric'] * 100:.0f} cm, Nav2 footprint {c['face'] * 100:.0f} cm{'  CONTACT' + (' (robot moving)' if d['active_contact'] else ' (robot stopped)') if d['contact'] else ''}  reached={d['reached']}")
            continue
        print(f"{c['label']:10s} {c['id']} {c['status']} {c['duration']:.1f} s  min mid-route clearance: metric {c['metric'] * 100:.1f} cm (to cell centre) | independent {c['face'] * 100:.1f} cm (to wall face)")
    import rclpy
    from builtin_interfaces.msg import Duration
    from geometry_msgs.msg import Point
    from nav_msgs.msg import OccupancyGrid
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from visualization_msgs.msg import Marker, MarkerArray
    rclpy.init()
    node = rclpy.create_node("replay_rviz")
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    map_pub, mk_pub = node.create_publisher(OccupancyGrid, "/map", latched), node.create_publisher(MarkerArray, "/replay/markers", 10)
    gcm_pub, wake_pub = node.create_publisher(OccupancyGrid, "/replay/global_costmap", 1), node.create_publisher(MarkerArray, "/replay/wake", 1)
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
        for gid, (gx, gy), gcol, gtxt in ((10 * n + 5, c["start"][:2], (0.3, 0.8, 1.0), "START"), (10 * n + 6, c["goal"][:2], (1.0, 0.2, 1.0), "GOAL")):
            g = Marker()
            g.header.frame_id, g.ns, g.id, g.type, g.action = "map", f"sg{n}", gid, Marker.CYLINDER, Marker.ADD
            g.pose.position.x, g.pose.position.y, g.pose.position.z, g.pose.orientation.w = float(gx), float(gy), 0.05, 1.0
            g.scale.x = g.scale.y = 0.5
            g.scale.z = 0.05
            g.color.r, g.color.g, g.color.b, g.color.a = *gcol, 0.8
            static.append(g)
            static.append(text(gid, f"sgt{n}", gx, gy + 0.5, gtxt, gcol, 0.3))
        mx, my, myaw = c["min_pose"]
        static.append(line(10 * n + 2, "minfp", rect(mx, my, myaw, *fp), rgb, 0.04))
        if c.get("walker"):
            d = c["dyn"]
            static.append(text(10 * n + 3, "label", mx, my + 0.6 + 0.35 * n, f"{c['label']} {c['id']}: gap {c['metric'] * 100:.0f} cm" + (" CONTACT" if d["contact"] else "") + ("" if d["reached"] else " NOT REACHED"), rgb))
            static.append(line(10 * n + 7, "walkerpath", list(zip(c["walker"][1], c["walker"][2])), rgb, 0.015, 0.5))
        else:
            static.append(text(10 * n + 3, "label", mx, my + 0.6 + 0.35 * n, f"{c['label']} {c['id']}: {c['metric'] * 100:.1f} cm (centre) / {c['face'] * 100:.1f} cm (face)", rgb))
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
    longest = max(max(c["t"][-1], c["gcm"][-1][0] if c.get("gcm") else 0.0) for c in cases)
    cc = next((c for c in cases if c.get("gcm")), None)      # the recorded costmap shown is the first case that has one
    static_flat, shown, last_case = grid.ravel(), [None], [None]
    spans = [max(c["t"][-1], c["gcm"][-1][0] if c.get("gcm") else 0.0) + pause * args.speed for c in cases]      # one pass of each case plus a pause
    dwall = (ndimage.distance_transform_edt(~mp["occ"]) * mp["res"])[::-1].ravel()      # metres to the nearest wall cell, in OccupancyGrid layout: marks scattered by lidar noise right next to a wall are not a wake
    try:
        while rclpy.ok() and (proc is None or proc.poll() is None):
            if args.sequence:      # one case at a time: its own robot, walker, plan and recorded costmap
                pos = ((time.time() - t_wall) * args.speed) % sum(spans)
                cur = next(i for i in range(len(cases)) if pos < sum(spans[:i + 1]))
                ph = pos - sum(spans[:cur])
                if cur != last_case[0]:
                    last_case[0], shown[0], cc = cur, None, (cases[cur] if cases[cur].get("gcm") else None)
            else:
                cur, ph = None, ((time.time() - t_wall) * args.speed) % (longest + pause * args.speed)
            msgs = [m for m in static if cur is None or m.id // 10 == cur]
            if cur is not None:
                msgs.append(text(900, "case", float(centre[0]), float(centre[1]) + span / 2 - 1.0, f"case {cur + 1}/{len(cases)}: {cases[cur]['label']} {cases[cur]['id']}", (1.0, 1.0, 1.0), 0.5))
            for n, (c, rgb) in enumerate(zip(cases, colours([c["label"] for c in cases]))):
                if cur is not None and n != cur:
                    continue
                tt = min(ph, c["t"][-1])
                x, y = np.interp(tt, c["t"], c["pose"][:, 0]), np.interp(tt, c["t"], c["pose"][:, 1])
                yaw = float(np.interp(tt, c["t"], np.unwrap(c["pose"][:, 2])))
                msgs.append(line(10 * n + 6, "robot", rect(x, y, yaw, *fp), rgb, 0.03))
                if args.zone and c.get("cm") is not None and c["label"].startswith("zona"):
                    st = next((e for e in reversed(c["cm"]) if e[0] <= tt), None)
                    act, who = (st[1], st[2]) if st else (0, "")
                    cy, sy = math.cos(yaw), math.sin(yaw)
                    for zi, zs in enumerate(args.zone):
                        parts = zs.split(",")
                        zf, zb, zw = float(parts[0]), float(parts[1]), float(parts[2])
                        mine = act == 2 and (len(parts) < 4 or who == parts[3])
                        col = (1.0, 0.1, 0.1) if mine else ((1.0, 0.85, 0.0) if act in (1, 3, 4) else (0.5, 0.6, 0.5))
                        zp = [(x + cy * u - sy * v, y + sy * u + cy * v) for u, v in ((zf, zw), (zf, -zw), (zb, -zw), (zb, zw), (zf, zw))]
                        msgs.append(line(10 * n + 9, f"zone{zi}", zp, col, 0.05 if mine else 0.02, 1.0 if mine else 0.5))
                    msgs.append(text(10 * n + 9, "zonetext", x, y - max(float(z.split(",")[2]) for z in args.zone) - 0.5, f"Collision Monitor: {['nothing', 'STOP', 'SLOWDOWN', 'APPROACH', 'LIMIT'][act]} {who}", (1.0, 0.1, 0.1) if act == 2 else ((1.0, 0.85, 0.0) if act else (0.5, 0.6, 0.5)), 0.3))
                if c.get("walker"):
                    o = walker_at(c["walker"], tt)
                    if o:
                        msgs.append(line(10 * n + 8, "walker", rect(*o, *c["walker"][4]), (0.0, 0.0, 0.0), 0.06))
            if cc is not None:
                payload = gcm_codec.frame_at(cc["gcm"], ph)
                if payload is not None and payload is not shown[0]:
                    shown[0] = payload
                    meta, data = cc["gcm_meta"], gcm_codec.decode(payload)
                    g = OccupancyGrid()
                    g.header.frame_id, g.info.resolution, g.info.width, g.info.height = "map", meta["res"], meta["width"], meta["height"]
                    g.info.origin.position.x, g.info.origin.position.y, g.info.origin.orientation.w = meta["origin"][0], meta["origin"][1], 1.0
                    g.data = data.tolist()
                    gcm_pub.publish(g)
                    ow = walker_at(cc["walker"], min(ph, cc["t"][-1])) if cc.get("walker") else None
                    wx, wy = ghost.wake_xy(data, static_flat, meta["width"], meta["height"], meta["origin"], meta["res"], [ow[:2]] if ow else [])
                    near_wall = dwall[np.floor((wy - meta["origin"][1]) / meta["res"]).astype(int) * meta["width"] + np.floor((wx - meta["origin"][0]) / meta["res"]).astype(int)] <= 0.6
                    wx, wy = wx[~near_wall], wy[~near_wall]
                    pts = Marker()
                    pts.header.frame_id, pts.ns, pts.id, pts.type, pts.action = "map", "wake", 0, Marker.SPHERE_LIST, Marker.ADD
                    pts.scale.x = pts.scale.y = pts.scale.z = 0.09
                    pts.color.r, pts.color.g, pts.color.b, pts.color.a = 1.0, 0.0, 1.0, 1.0
                    pts.points = [Point(x=float(x), y=float(y), z=0.12) for x, y in zip(wx, wy)]
                    lab = Marker()
                    lab.header.frame_id, lab.ns, lab.id, lab.type, lab.action = "map", "wake", 1, Marker.TEXT_VIEW_FACING, Marker.ADD
                    lab.pose.position.x, lab.pose.position.y, lab.pose.position.z = float(meta["origin"][0] + meta["width"] * meta["res"] / 2), float(meta["origin"][1] + meta["height"] * meta["res"] - 1.0), 0.5
                    lab.scale.z, lab.color.r, lab.color.g, lab.color.b, lab.color.a = 0.4, 1.0, 0.4, 1.0, 1.0
                    lab.text = f"{cc['label']} {cc['id']}  t={ph:5.1f} s   lethal cells with no wall or walker to explain them (magenta): {len(wx)}"
                    pts.lifetime = lab.lifetime = Duration(sec=1)
                    wake_pub.publish(MarkerArray(markers=[pts, lab]))
            for mk in msgs:
                mk.lifetime = Duration(sec=0, nanosec=500_000_000)      # markers of the case that just ended expire on their own
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
