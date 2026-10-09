#!/usr/bin/env python3
"""Lightweight sensor-side ROS topic counter for exploratory AMCL runs."""
import json
import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.msg import ParticleCloud
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


def stamp_ns(msg):
    return int(msg.header.stamp.sec)*1_000_000_000+int(msg.header.stamp.nanosec)


class Probe(Node):
    def __init__(self):
        super().__init__('amcl_topic_probe')
        self.path=str(self.declare_parameter('output_jsonl','/tmp/amcl-topic-probe.jsonl').value)
        self.records=[]
        self.saved=0
        scan_qos=QoSProfile(depth=1000,reliability=ReliabilityPolicy.BEST_EFFORT)
        # ParticleCloud grows with the AMCL population. A deep DDS history can
        # reserve excessive memory and obscure transport failures at 5k+.
        cloud_qos=QoSProfile(depth=16,reliability=ReliabilityPolicy.BEST_EFFORT)
        pose_qos=QoSProfile(depth=1000,reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(LaserScan,'/scan',self.scan,scan_qos)
        self.create_subscription(ParticleCloud,'/particle_cloud',self.cloud,cloud_qos)
        self.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',self.pose,pose_qos)
        self.create_timer(1.0,self.report)

    def scan(self,msg):
        self.records.append({'topic':'/scan','stamp_ns':stamp_ns(msg),
                             'finite_beams':sum(1 for x in msg.ranges if math.isfinite(x))})

    def cloud(self,msg):
        self.records.append({'topic':'/particle_cloud','stamp_ns':stamp_ns(msg),
                             'particles':len(msg.particles)})

    def pose(self,msg):
        self.records.append({'topic':'/amcl_pose','stamp_ns':stamp_ns(msg)})

    def report(self):
        totals={topic:sum(1 for r in self.records if r['topic']==topic)
                for topic in ('/scan','/particle_cloud','/amcl_pose')}
        self.get_logger().info('topic probe '+json.dumps(totals,sort_keys=True))
        self.flush_records()

    def flush_records(self):
        if self.saved>=len(self.records):return
        with open(self.path,'a',encoding='utf-8') as f:
            for row in self.records[self.saved:]:
                f.write(json.dumps(row,sort_keys=True)+'\n')
        self.saved=len(self.records)

    def save(self):
        self.flush_records()


def main():
    rclpy.init();node=Probe()
    try:rclpy.spin(node)
    finally:node.save();node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
