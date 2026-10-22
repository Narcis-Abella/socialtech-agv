"""Global planner benchmark without moving the robot: asks each planner of planner_server for the path of every start/goal pair of a scenario file (ComputePathToPose with an explicit start)
and stores the planning time reported by the server (result.planning_time), wall time of the request, the error code and the path. Run inside the sim image with the simulation and Nav2 up (run_trials.sh with RUNNER=nav_sim/plan_bench.py).
usage: plan_bench.py <scenario.json> <out dir> [reps] --ros-args -p use_sim_time:=true      env: PLANNERS (comma separated planner_plugins ids, default all in the params), WARMUP (2 requests per planner)"""
import json
import os
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from rcl_interfaces.srv import GetParameters
from rclpy.parameter import Parameter


def pose(n, x, y, yaw):
    import math
    p = PoseStamped()
    p.header.frame_id, p.header.stamp = "map", n.get_clock().now().to_msg()
    p.pose.position.x, p.pose.position.y = x, y
    p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return p


def planner_ids(n):
    c = n.create_client(GetParameters, "/planner_server/get_parameters")
    c.wait_for_service(timeout_sec=10.0)
    req = GetParameters.Request()
    req.names = ["planner_plugins"]
    f = c.call_async(req)
    rclpy.spin_until_future_complete(n, f, timeout_sec=10.0)
    return list(f.result().values[0].string_array_value)


def main(scenario, out_dir, reps=1):
    rclpy.init(args=sys.argv)
    n = rclpy.create_node("plan_bench")
    os.makedirs(out_dir, exist_ok=True)
    client = ActionClient(n, ComputePathToPose, "compute_path_to_pose")
    client.wait_for_server()
    ids = os.environ.get("PLANNERS", "").split(",") if os.environ.get("PLANNERS") else planner_ids(n)
    trials = json.load(open(scenario))["trials"]
    rows = []
    for pid in ids:
        for k in range(-int(os.environ.get("WARMUP", 2)), int(reps) * len(trials)):
            t = trials[k % len(trials)] if k >= 0 else trials[0]
            g = ComputePathToPose.Goal()
            g.start, g.goal, g.use_start, g.planner_id = pose(n, *t["start"]), pose(n, *t["goal"]), True, pid
            t0 = time.perf_counter()
            sent = client.send_goal_async(g)
            rclpy.spin_until_future_complete(n, sent)
            h = sent.result()
            res = h.get_result_async()
            rclpy.spin_until_future_complete(n, res)
            wall = time.perf_counter() - t0
            if k < 0:
                continue
            r = res.result().result
            ok = bool(r.path.poses)
            rows.append({"id": t["id"], "planner": pid, "rep": k // len(trials), "ok": ok, "error_code": int(getattr(r, "error_code", -1)),
                         "planning_time": r.planning_time.sec + r.planning_time.nanosec * 1e-9, "wall_time": wall,
                         "path": [[round(p.pose.position.x, 3), round(p.pose.position.y, 3)] for p in r.path.poses]})
        print(f"{pid}: {sum(r['ok'] for r in rows if r['planner'] == pid)} of {sum(r['planner'] == pid for r in rows)} paths", flush=True)
    json.dump({"planners": ids, "rows": rows}, open(os.path.join(out_dir, "plans.json"), "w"))
    rclpy.shutdown()


if __name__ == "__main__":
    main(*(sys.argv[1:sys.argv.index("--ros-args")] if "--ros-args" in sys.argv else sys.argv[1:]))
