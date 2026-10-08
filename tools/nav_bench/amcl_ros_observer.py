#!/usr/bin/env python3
"""ROS adapter for the sensor/map-only AMCL acceptance gate.

This process subscribes only to scan, map, AMCL pose and weighted particles. It
must never be given a reference, inventory CSV, map trajectory or evaluator path.
"""
import json
import math
import time
from collections import deque
from std_srvs.srv import Empty

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.msg import ParticleCloud
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

from amcl_acceptance import AcceptanceTracker, Occupancy, Scan, evaluate, particle_modes


def _yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


class Observer(Node):
    def __init__(self):
        super().__init__('amcl_acceptance_observer')
        self.declare_parameter('scenario','S2')
        self.declare_parameter('output_jsonl','/tmp/amcl-observer.jsonl')
        self.declare_parameter('timeout_s',60.0)
        self.scenario=self.get_parameter('scenario').value
        self.tracker=AcceptanceTracker(self.scenario)
        self.path=self.get_parameter('output_jsonl').value
        self.timeout_s=float(self.get_parameter('timeout_s').value)
        self.timeout_reported=False
        self.map=None;self.pose=None;self.pose_stamp=None;self.particles=None;self.particle_stamp=None
        self.started=None;self.acceptance=None;self.pending_scans=deque();self.latest_fit_stamp=None
        self.fit_tracker=AcceptanceTracker(self.scenario);self.check_stamps=[];self.last_gate_pass=False
        map_qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid,'/map',self.on_map,map_qos)
        self.create_subscription(ParticleCloud,'/particle_cloud',self.on_particles,qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',self.on_pose,10)
        self.update_client=self.create_client(Empty,'/request_nomotion_update')
        self.create_subscription(LaserScan,'/scan',lambda m:self.on_scan(m,False),qos_profile_sensor_data)
        self.create_subscription(LaserScan,'/check_scan',lambda m:self.on_scan(m,True),qos_profile_sensor_data)
        self.create_timer(.02,self.drain_scans)
        self.create_timer(.1,self.check_deadline)

    def on_map(self,m):
        o=m.info.origin.position;q=m.info.origin.orientation
        self.map=Occupancy(np.asarray(m.data,dtype=np.int16).reshape((m.info.height,m.info.width)),
                           m.info.resolution,o.x,o.y,_yaw(q),65)

    def on_particles(self,m):
        self.particles=[(p.pose.position.x,p.pose.position.y,_yaw(p.pose.orientation),p.weight) for p in m.particles]
        self.particle_stamp=m.header.stamp.sec*1_000_000_000+m.header.stamp.nanosec
        # AMCL publishes this only after it has processed a scan. Requesting
        # the next stationary update here avoids racing its scan callback.
        if self.update_client.service_is_ready():
            self.update_client.call_async(Empty.Request())

    def on_pose(self,m):
        p=m.pose.pose.position;q=m.pose.pose.orientation
        self.pose=(p.x,p.y,_yaw(q))
        self.pose_stamp=m.header.stamp.sec*1_000_000_000+m.header.stamp.nanosec

    def on_scan(self,m,is_check):
        now=time.monotonic()
        if self.started is None:self.started=now
        stamp=m.header.stamp.sec*1_000_000_000+m.header.stamp.nanosec
        # S2 confirmation uses distinct, post-update batches from the same
        # 3 s stream. The protocol does not prescribe a withheld S2 partition.
        # S1 continues to use its explicit same-frame 70/30 check scan.
        if self.scenario=='S2' and not is_check:is_check=True
        if not is_check:self.latest_fit_stamp=stamp
        elif self.scenario=='S2':self.latest_fit_stamp=stamp
        self.pending_scans.append((m,is_check,stamp,now-self.started))

    def drain_scans(self):
        # AMCL and this observer receive the same scan independently. Wait for
        # pose and particle messages stamped at/after that scan before scoring,
        # so the diagnostic cannot accidentally reuse the previous update.
        while self.pending_scans:
            m,is_check,stamp,elapsed=self.pending_scans[0]
            required_stamp=self.latest_fit_stamp if is_check else stamp
            if (self.particles is None or self.map is None or self.particle_stamp is None
                    or required_stamp is None or self.particle_stamp<required_stamp):
                break
            self.pending_scans.popleft()
            self.score_scan(m,is_check,stamp,elapsed)

    def score_scan(self,m,is_check,stamp,elapsed):
        now=time.monotonic()
        record={'sensor_stamp_ns':stamp,'wall_elapsed_s':elapsed,
                'finite_beams':int(np.isfinite(np.asarray(m.ranges,dtype=float)).sum()),
                'input_kind':'check' if is_check else 'fit',
                'have_map':self.map is not None,'have_pose':self.pose is not None,
                'pose_stamp_ns':self.pose_stamp,'particle_stamp_ns':self.particle_stamp,
                'particle_count':len(self.particles) if self.particles is not None else 0}
        modes=particle_modes(self.particles,5) if self.particles else []
        # Use the strongest AMCL particle mode as the live hypothesis. The
        # lower-rate /amcl_pose publication is logged separately and is not
        # copied forward as if it were a fresh scan-aligned estimate.
        pose=(modes[0].x,modes[0].y,modes[0].yaw) if modes else None
        record['amcl_pose']=self.pose
        record['amcl_pose_stamp_ns']=self.pose_stamp
        if self.map is not None and pose is not None and self.particles:
            scan=Scan(np.asarray(m.ranges,dtype=float),m.angle_min,m.angle_increment,m.range_min,m.range_max)
            result=evaluate(scan,self.particles,pose,self.map)
            record.update({'gate_passes':result.passes,'gate_reason':result.reason,
                           'particle_concentration':result.particle_concentration,
                           'primary_support':result.primary_support,'runner_up_support':result.runner_up_support,
                           'beam_span_rad':result.beam_span_rad,'pose':pose})
            self.last_gate_pass=result.passes
            if is_check:
                if result.passes:self.check_stamps.append(stamp)
                else:self.check_stamps=[]
            event=self.fit_tracker.observe(stamp,elapsed,pose,result)
            if event and is_check and len(self.check_stamps)>=2 and self.check_stamps[-1]-self.check_stamps[0]>=500_000_000:
                self.acceptance={'kind':'accept','sensor_stamp_ns':stamp,
                                 'wall_elapsed_s':elapsed,'pose':pose,
                                 'confirmation_count':len(self.check_stamps)}
                record['acceptance_event']=self.acceptance
        with open(self.path,'a',encoding='utf-8') as f:
            f.write(json.dumps(record,sort_keys=True,allow_nan=False)+'\n')
        if self.acceptance:
            self.get_logger().info('observable acceptance gate passed')

    def check_deadline(self):
        if self.started is None or self.acceptance is not None or self.timeout_reported:
            return
        elapsed=time.monotonic()-self.started
        if elapsed<self.timeout_s:return
        event={'terminal_event':'timeout','wall_elapsed_s':elapsed,'timeout_s':self.timeout_s}
        with open(self.path,'a',encoding='utf-8') as f:
            f.write(json.dumps(event,sort_keys=True,allow_nan=False)+'\n')
        self.timeout_reported=True
        self.get_logger().info('observable acceptance deadline reached')


def main():
    rclpy.init();node=Observer()
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
