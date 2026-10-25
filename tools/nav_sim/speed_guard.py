"""Speed limit from what the local costmap sees: the robot may only go as fast as it can still stop in the free distance ahead of it, along a lane as wide as its footprint plus a margin.
Reads /local_costmap/costmap (lethal cells), publishes nav2_msgs/SpeedLimit on `speed_limit`, which controller_server hands to the RPP (setSpeedLimit: absolute limit becomes desired_linear_vel; 0.0 = no limit).
  speed_guard.py --ros-args -p use_sim_time:=true [-p decel:=1.0 -p margin:=0.15 -p lane_margin:=0.10 -p front:=0.35 -p half_width:=0.29 -p horizon:=1.5 -p v_min:=0.05]
Pure functions first (tested on a laptop without ROS); the node below imports ROS only when run."""
import math

import numpy as np


def free_distance(points, half_width, front, horizon):
    """Distance (m) from the robot front to the nearest cell in the lane |y| <= half_width, x up to `horizon` from the front. points: (N, 2) robot frame, x forward, y left.
    A cell in the lane that is already closer than the front counts as 0.0; inf when the lane is clear."""
    p = np.asarray(points, float).reshape(-1, 2)
    inlane = (np.abs(p[:, 1]) <= half_width) & (p[:, 0] >= 0.0) & (p[:, 0] <= front + horizon)
    if not inlane.any():
        return math.inf
    return max(0.0, float(p[inlane, 0].min()) - front)


def speed_limit(d_free, decel, margin, v_min):
    """Fastest speed that still stops `margin` before the obstacle at deceleration `decel`; None without an obstacle. Never below v_min: Nav2 reads 0.0 as 'no limit'."""
    if math.isinf(d_free):
        return None
    return max(v_min, math.sqrt(2.0 * decel * max(d_free - margin, 0.0)))


def main(argv):
    import rclpy
    from nav2_msgs.msg import SpeedLimit
    from nav_msgs.msg import OccupancyGrid
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformException, TransformListener
    rclpy.init(args=argv)
    node = rclpy.create_node("speed_guard")
    P = {k: node.declare_parameter(k, v).value for k, v in dict(decel=1.0, margin=0.15, lane_margin=0.10, front=0.35, half_width=0.29, horizon=1.5, v_min=0.05, hz=20.0, frame="base_footprint").items()}
    tf = Buffer()
    TransformListener(tf, node)
    pub = node.create_publisher(SpeedLimit, "speed_limit", 10)
    state = {"grid": None, "cells": None, "last": None, "t": 0.0}

    def on_grid(m):
        a = np.asarray(m.data, np.int16).reshape(m.info.height, m.info.width)
        rows, cols = np.nonzero(a == 100)                      # lethal in OccupancyGrid terms (Nav2 publishes 254 as 100)
        r = m.info.resolution
        state["cells"] = np.c_[m.info.origin.position.x + (cols + 0.5) * r, m.info.origin.position.y + (rows + 0.5) * r]
        state["frame"] = m.header.frame_id

    node.create_subscription(OccupancyGrid, "/local_costmap/costmap", on_grid, 1)

    def tick():
        if state["cells"] is None:
            return
        try:
            t = tf.lookup_transform(P["frame"], state["frame"], Time()).transform
        except TransformException:
            return
        yaw = 2.0 * math.atan2(t.rotation.z, t.rotation.w)
        c, s = math.cos(yaw), math.sin(yaw)
        xy = state["cells"]
        robot = np.c_[c * xy[:, 0] - s * xy[:, 1] + t.translation.x, s * xy[:, 0] + c * xy[:, 1] + t.translation.y]
        d = free_distance(robot, P["half_width"] + P["lane_margin"], P["front"], P["horizon"])
        v = speed_limit(d, P["decel"], P["margin"], P["v_min"])
        v = 0.0 if v is None else round(v, 2)                  # 0.0 = NO_SPEED_LIMIT
        now = node.get_clock().now().nanoseconds / 1e9
        if v != state["last"] or now - state["t"] > 1.0:       # on change, and a heartbeat every second
            m = SpeedLimit()
            m.percentage, m.speed_limit = False, float(v)
            pub.publish(m)
            state["last"], state["t"] = v, now

    node.create_timer(1.0 / P["hz"], tick)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    import sys
    main(sys.argv)
