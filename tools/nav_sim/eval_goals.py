"""Sends the goals of goals.yaml to Nav2 in the simulation and scores each run.
  eval_goals.py <goals.yaml> <map.yaml> <out.json>      (inside the sim image, with the sim and Nav2 running)
Per goal: reached or not, time, closest approach to a wall and localization error, both against the pose Gazebo publishes for the robot (/gt/pose; /odom_raw is wheel odometry with ~6 % slip, not ground truth).
The map frame is the world frame: the map is built with odom_source:=ground_truth, whose odom frame is the world one, and slam_toolbox starts `map` on `odom`."""
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


def robot_xy(buffer):
    """(x, y) of base_footprint in map from TF, or None if the transform is not there yet. It is AMCL's map->odom composed with the odometry, so it follows the robot at the odometry rate; /amcl_pose is published only after 0.25 m of motion and would lag the ground truth by up to that."""
    from rclpy.time import Time
    from tf2_ros import TransformException
    try:
        t = buffer.lookup_transform("map", "base_footprint", Time()).transform.translation
    except TransformException:
        return None
    return (t.x, t.y)


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
    from geometry_msgs.msg import PoseStamped
    from action_msgs.msg import GoalStatus
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient
    from tf2_ros import Buffer, TransformListener

    rclpy.init()
    nav = rclpy.create_node("eval_goals")
    client = ActionClient(nav, NavigateToPose, "navigate_to_pose")
    state = {"gt": None, "min_clr": float("inf")}
    grid, res, origin = load_grid(map_yaml)

    def on_gt(m):
        state["gt"] = (m.pose.position.x, m.pose.position.y)
        state["min_clr"] = min(state["min_clr"], clearance(grid, res, origin, *state["gt"]))

    nav.create_subscription(PoseStamped, "/gt/pose", on_gt, 10)
    tf = Buffer()
    TransformListener(tf, nav)
    client.wait_for_server()  # run_eval.sh has already waited for bt_navigator to be active
    rows = []
    for g in yaml.safe_load(open(goals_yaml))["goals"]:
        p = PoseStamped()
        p.header.frame_id = "map"
        p.header.stamp = nav.get_clock().now().to_msg()
        p.pose.position.x, p.pose.position.y = g["x"], g["y"]
        p.pose.orientation.z, p.pose.orientation.w = math.sin(g["yaw"] / 2), math.cos(g["yaw"] / 2)
        state["min_clr"] = float("inf")
        t0 = nav.get_clock().now()
        goal = NavigateToPose.Goal()
        goal.pose = p
        sent = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(nav, sent)
        handle = sent.result()
        ok = False
        if handle.accepted:
            result = handle.get_result_async()
            while not result.done():
                rclpy.spin_once(nav, timeout_sec=0.1)
            ok = result.result().status == GoalStatus.STATUS_SUCCEEDED
        dt = (nav.get_clock().now() - t0).nanoseconds / 1e9
        est = robot_xy(tf)
        err = math.hypot(est[0] - state["gt"][0], est[1] - state["gt"][1]) if est and state["gt"] else float("nan")
        rows.append(dict(goal=g, ok=ok, t=dt, clr=state["min_clr"], err=err))
        print(rows[-1], flush=True)
    json.dump({"rows": rows, "summary": summarize(rows)}, open(out_json, "w"), indent=1)
    print(summarize(rows))
    rclpy.shutdown()


if __name__ == "__main__":
    main(*sys.argv[1:4])
