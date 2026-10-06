"""Sends the goals of goals.yaml to Nav2 in the simulation and scores each run.
  eval_goals.py <goals.yaml> <map.yaml> <out.json>      (inside the sim image, with the sim and Nav2 running)
Per goal: reached or not, time, closest approach to a wall (ground truth = /odom_raw, whose origin is the map origin by construction), AMCL error against it."""
import json
import math
import os
import sys

import numpy as np
import yaml
from PIL import Image


def clearance(grid, res, origin, x, y, radius=1.0):
    """Minimum distance (m) from (x, y) to the centre of an occupied cell; inf if none within `radius`. grid[row, col], row 0 = top of the image."""
    h = grid.shape[0]
    rows, cols = np.nonzero(grid)
    if rows.size == 0:
        return float("inf")
    cx = origin[0] + (cols + 0.5) * res
    cy = origin[1] + (h - rows - 0.5) * res
    d = np.hypot(cx - x, cy - y)
    d = d[d <= radius]
    return float(d.min()) if d.size else float("inf")


def summarize(rows):
    """rows: dicts with ok, t (s), clr (m), err (m). Times are those of the goals reached."""
    t = [r["t"] for r in rows if r["ok"]]
    err = [r["err"] for r in rows]
    return {
        "n": len(rows), "succeeded": sum(r["ok"] for r in rows),
        "time_median": float(np.median(t)) if t else None, "time_p95": float(np.percentile(t, 95)) if t else None,
        "time_max": max(t) if t else None,
        "clearance_min": min(r["clr"] for r in rows),
        "err_median": float(np.median(err)), "err_p95": float(np.percentile(err, 95)), "err_max": max(err),
    }


def load_grid(map_yaml):
    m = yaml.safe_load(open(map_yaml))
    img = np.asarray(Image.open(os.path.join(os.path.dirname(map_yaml), m["image"])).convert("L"))
    occ = img < (1.0 - m["occupied_thresh"]) * 255 if m.get("negate", 0) == 0 else img > m["occupied_thresh"] * 255
    return occ, m["resolution"], m["origin"][:2]


def main(goals_yaml, map_yaml, out_json):
    import rclpy
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
    from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
    from nav_msgs.msg import Odometry

    rclpy.init()
    nav = BasicNavigator()
    state = {"gt": None, "amcl": None, "min_clr": float("inf")}
    grid, res, origin = load_grid(map_yaml)

    def on_gt(m):
        state["gt"] = (m.pose.pose.position.x, m.pose.pose.position.y)
        state["min_clr"] = min(state["min_clr"], clearance(grid, res, origin, *state["gt"]))

    nav.create_subscription(Odometry, "/odom_raw", on_gt, 10)
    nav.create_subscription(PoseWithCovarianceStamped, "/amcl_pose",
                            lambda m: state.update(amcl=(m.pose.pose.position.x, m.pose.pose.position.y)), 10)
    nav.waitUntilNav2Active()
    rows = []
    for g in yaml.safe_load(open(goals_yaml))["goals"]:
        p = PoseStamped()
        p.header.frame_id = "map"
        p.pose.position.x, p.pose.position.y = g["x"], g["y"]
        p.pose.orientation.z, p.pose.orientation.w = math.sin(g["yaw"] / 2), math.cos(g["yaw"] / 2)
        state["min_clr"] = float("inf")
        t0 = nav.get_clock().now()
        nav.goToPose(p)
        while not nav.isTaskComplete():
            rclpy.spin_once(nav, timeout_sec=0.1)
        ok = nav.getResult() == TaskResult.SUCCEEDED
        dt = (nav.get_clock().now() - t0).nanoseconds / 1e9
        err = math.hypot(state["amcl"][0] - state["gt"][0], state["amcl"][1] - state["gt"][1]) if state["amcl"] and state["gt"] else float("nan")
        rows.append(dict(goal=g, ok=ok, t=dt, clr=state["min_clr"], err=err))
        print(rows[-1], flush=True)
    json.dump({"rows": rows, "summary": summarize(rows)}, open(out_json, "w"), indent=1)
    print(summarize(rows))
    rclpy.shutdown()


if __name__ == "__main__":
    main(*sys.argv[1:4])
