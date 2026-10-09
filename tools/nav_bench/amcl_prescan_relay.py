#!/usr/bin/env python3
"""Serialize stationary scans behind AMCL's no-motion update service.

This executor sees only LaserScan messages and AMCL acknowledgements. It never
reads an inventory, map, reference, initial pose or evaluation result.
"""
from collections import deque
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty


def stamp_ns(message):
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


class PrescanRelay(Node):
    def __init__(self):
        super().__init__('amcl_prescan_relay')
        qos=QoSProfile(depth=100,reliability=ReliabilityPolicy.RELIABLE)
        self.queue=deque()
        self.state='idle'
        self.waiting_stamp=None
        self.waiting_since=None
        self.received=self.published=self.acknowledged=0
        self.client=self.create_client(Empty,'/request_nomotion_update')
        self.pub=self.create_publisher(LaserScan,'/scan',qos)
        self.create_subscription(LaserScan,'/candidate_scan',self.on_scan,qos)
        self.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',self.on_pose,qos)
        self.create_timer(.01,self.tick)
        self.create_timer(1.,self.report)

    def on_scan(self,message):
        if self.queue and stamp_ns(message)<=stamp_ns(self.queue[-1]):
            self.get_logger().error('nonincreasing candidate scan timestamp')
            return
        self.queue.append(message)
        self.received+=1

    def tick(self):
        if self.state=='waiting_pose' and time.monotonic()-self.waiting_since>5.:
            self.get_logger().error(f'AMCL pose acknowledgement timed out for {self.waiting_stamp}')
            self.state='blocked'
        if self.state!='idle' or not self.queue or not self.client.service_is_ready():
            return
        self.state='requesting'
        self.client.call_async(Empty.Request()).add_done_callback(self.on_response)

    def on_response(self,future):
        try:future.result()
        except Exception as exc:
            self.get_logger().error(f'no-motion request failed: {exc}')
            self.state='blocked'
            return
        message=self.queue.popleft()
        self.waiting_stamp=stamp_ns(message)
        self.waiting_since=time.monotonic()
        self.pub.publish(message)
        self.published+=1
        self.state='waiting_pose'

    def on_pose(self,message):
        if self.state!='waiting_pose' or stamp_ns(message)!=self.waiting_stamp:
            return
        self.acknowledged+=1
        self.waiting_stamp=None
        self.state='idle'

    def report(self):
        self.get_logger().info(f'prescan relay received={self.received} '
            f'published={self.published} pose_acks={self.acknowledged} '
            f'queued={len(self.queue)} state={self.state}')


def main():
    rclpy.init();node=PrescanRelay()
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
