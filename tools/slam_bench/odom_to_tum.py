"""Write nav_msgs/Odometry messages of one topic to a TUM file; exit once the topic is silent for --idle s
(after the first message), or after --max-wait s without any.
usage: odom_to_tum.py <topic> <out.tum> [--idle 8] [--max-wait 60]"""
import argparse
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

ap = argparse.ArgumentParser()
ap.add_argument("topic")
ap.add_argument("out")
ap.add_argument("--idle", type=float, default=8.0)
ap.add_argument("--max-wait", type=float, default=60.0)
a = ap.parse_args()

rclpy.init()
node = Node("odom_to_tum")
f = open(a.out, "w")
state = {"n": 0, "last": time.monotonic()}


def cb(m):
    p, q, s = m.pose.pose.position, m.pose.pose.orientation, m.header.stamp
    f.write(f"{s.sec + s.nanosec * 1e-9:.9f} {p.x} {p.y} {p.z} {q.x} {q.y} {q.z} {q.w}\n")
    f.flush()
    state["n"] += 1
    state["last"] = time.monotonic()


node.create_subscription(Odometry, a.topic, cb, 1000)
start = time.monotonic()
while rclpy.ok():
    rclpy.spin_once(node, timeout_sec=0.5)
    now = time.monotonic()
    if (state["n"] and now - state["last"] > a.idle) or (not state["n"] and now - start > a.max_wait):
        break
print(f"{state['n']} poses written to {a.out}")
f.close()
