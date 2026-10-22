#!/usr/bin/env python3
"""Request AMCL's supported stationary-scan update after each input scan.

The request is delayed so AMCL can consume the scan before its next update is
forced. This is an executor aid for stationary S2 playback, never an input to
the acceptance observer or localization method.
"""
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty


class NoMotionDriver(Node):
    def __init__(self):
        super().__init__('amcl_nomotion_driver')
        self.client=self.create_client(Empty,'/request_nomotion_update')
        self.delay_s=float(self.declare_parameter('delay_s',0.05).value)
        self.trigger=str(self.declare_parameter('trigger','scan_delay').value)
        if self.trigger not in ('scan_delay','pose_ack'):
            raise ValueError('trigger must be scan_delay or pose_ack')
        self.pending_until=None
        self.requests=0
        self.scans=0
        self.pose_acks=0
        self.last_pose_stamp=None
        self.create_subscription(LaserScan,'/scan',self.on_scan,10)
        if self.trigger=='pose_ack':
            self.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',self.on_pose,10)
        self.create_timer(0.01,self.on_timer)
        self.create_timer(1.0,self.report)

    def on_scan(self,_msg):
        self.scans+=1
        if self.trigger=='scan_delay':
            self.pending_until=time.monotonic()+self.delay_s

    def on_pose(self,msg):
        # Nav2 publishes /amcl_pose after the ParticleCloud branch in its scan
        # callback. Acknowledging the pose avoids setting force_update_ while
        # the current (large-particle) scan is still being processed.
        stamp=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        if self.last_pose_stamp is not None and stamp<=self.last_pose_stamp:
            return
        self.last_pose_stamp=stamp
        self.pose_acks+=1
        self.pending_until=time.monotonic()

    def on_timer(self):
        if self.pending_until is None or time.monotonic()<self.pending_until:
            return
        if not self.client.service_is_ready():
            return
        future=self.client.call_async(Empty.Request())
        future.add_done_callback(self.on_response)
        self.pending_until=None

    def on_response(self,future):
        try:
            future.result()
            self.requests+=1
        except Exception as exc:  # surfaced as an operational diagnostic
            self.get_logger().error(f'no-motion update request failed: {exc}')

    def report(self):
        self.get_logger().info(
            f"stationary scan driver scans={self.scans} requests={self.requests} "
            f"pose_acks={self.pose_acks} trigger={self.trigger} "
            f"pending={self.pending_until is not None}")


def main():
    rclpy.init();node=NoMotionDriver()
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
