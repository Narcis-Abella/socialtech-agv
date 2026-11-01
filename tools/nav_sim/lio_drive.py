"""Scripted /cmd_vel for the full-LIO simulation: a leg forward, a half turn, the same leg back, a half turn (the robot ends where it began).
usage: lio_drive.py [leg_m=5] [v=0.4] [w=0.5]      sim time; waits 5 s first (FAST-LIO2 takes gravity from the still IMU)."""
import math
import sys

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


def plan(leg, v, w):
    """[(seconds, linear, angular)]"""
    return [(5., 0., 0.), (leg / v, v, 0.), (math.pi / w, 0., w), (leg / v, v, 0.), (math.pi / w, 0., w), (3., 0., 0.)]


class Drive(Node):
    def __init__(self, steps):
        super().__init__('lio_drive')
        self.pub, self.steps, self.t0 = self.create_publisher(Twist, '/cmd_vel', 10), steps, None
        self.create_timer(.05, self.tick)

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        self.t0 = self.t0 if self.t0 is not None else now
        t, m = now - self.t0, Twist()
        for dur, v, w in self.steps:
            if t < dur:
                m.linear.x, m.angular.z = v, w
                break
            t -= dur
        else:
            raise SystemExit
        self.pub.publish(m)


if __name__ == '__main__':
    a = [float(x) for x in sys.argv[1:]]
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    try:
        rclpy.spin(Drive(plan(*a)))
    except SystemExit:
        pass
