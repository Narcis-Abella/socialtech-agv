"""Surveys the simulated world from known poses: for each (x, y) of the survey file the robot is put there (Gazebo set_pose), left to settle, and one noise-free /scan_raw is stored with the
pose of the LiDAR in the world (ground-truth base pose composed with the static base_footprint -> LiDAR transform). Build the map with gt_map.py.
usage (inside the sim image, simulation running): gt_mapper.py <survey.json> <out.json> --ros-args -p use_sim_time:=true      env: ROBOT_NAME (tracer_robot), WORLD (loyola), SCAN (/scan_raw), LIDAR_FRAME (rslidar_link)"""
import json
import math
import os
import subprocess
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


def main(survey, out):
    rclpy.init(args=sys.argv)
    n = rclpy.create_node("gt_mapper")
    tf, state = Buffer(), {"gt": None, "scan": None, "stamp": 0}
    TransformListener(tf, n)
    n.create_subscription(PoseStamped, "/gt/pose", lambda m: state.update(gt=(m.pose.position.x, m.pose.position.y, 2 * math.atan2(m.pose.orientation.z, m.pose.orientation.w))), 10)
    n.create_subscription(LaserScan, os.environ.get("SCAN", "/scan_raw"), lambda m: state.update(scan=m, stamp=state["stamp"] + 1), qos_profile_sensor_data)

    def spin_for(sec):
        end = n.get_clock().now().nanoseconds / 1e9 + sec
        while n.get_clock().now().nanoseconds / 1e9 < end:
            rclpy.spin_once(n, timeout_sec=0.02)

    while state["gt"] is None or state["scan"] is None:
        rclpy.spin_once(n, timeout_sec=0.1)
    try:
        lt = tf.lookup_transform("base_footprint", os.environ.get("LIDAR_FRAME", "rslidar_link"), Time(), rclpy.duration.Duration(seconds=5.0)).transform
    except Exception as e:
        raise SystemExit(f"no base_footprint -> LiDAR transform: {e}")
    off = (lt.translation.x, lt.translation.y, 2 * math.atan2(lt.rotation.z, lt.rotation.w))
    print("lidar in base_footprint:", off, flush=True)
    scans = []
    for k, (x, y) in enumerate(json.load(open(survey))["poses"]):
        req = f'name: "{os.environ.get("ROBOT_NAME", "tracer_robot")}" position {{x: {x} y: {y} z: 0.15}} orientation {{w: 1}}'
        subprocess.run(["gz", "service", "-s", f"/world/{os.environ.get('WORLD', 'loyola')}/set_pose", "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean", "--timeout", "3000", "--req", req],
                       check=True, capture_output=True)
        spin_for(1.0)
        seen = state["stamp"]
        while state["stamp"] < seen + 2:          # a scan that began after the robot settled
            rclpy.spin_once(n, timeout_sec=0.05)
        gx, gy, gyaw = state["gt"]
        s = state["scan"]
        lx, ly = gx + off[0] * math.cos(gyaw) - off[1] * math.sin(gyaw), gy + off[0] * math.sin(gyaw) + off[1] * math.cos(gyaw)
        scans.append({"pose": [round(lx, 4), round(ly, 4), round(gyaw + off[2], 5)], "base_gt": [gx, gy, gyaw], "angle_min": s.angle_min, "angle_increment": s.angle_increment,
                      "range_min": s.range_min, "range_max": s.range_max, "ranges": [round(r, 4) if math.isfinite(r) else float("inf") for r in s.ranges]})
        if k % 20 == 0:
            print(f"{k} poses, last at ({x}, {y}): base_gt ({gx:.2f}, {gy:.2f})", flush=True)
    json.dump({"lidar_in_base": off, "scans": scans}, open(out, "w"))
    rclpy.shutdown()


if __name__ == "__main__":
    main(*(sys.argv[1:sys.argv.index("--ros-args")] if "--ros-args" in sys.argv else sys.argv[1:]))
