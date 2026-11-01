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
from sensor_msgs.msg import Imu, LaserScan

from amcl_acceptance import AcceptanceTracker, Occupancy, Scan, causal_cloud_pose_pair, evaluate, particle_modes, scan_update_state


def _yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


class Observer(Node):
    def __init__(self):
        super().__init__('amcl_acceptance_observer')
        self.declare_parameter('scenario','S2')
        self.declare_parameter('output_jsonl','/tmp/amcl-observer.jsonl')
        self.declare_parameter('timeout_s',60.0)
        self.declare_parameter('input_start_ns',0)
        self.declare_parameter('drive_nomotion_updates',True)
        self.scenario=self.get_parameter('scenario').value
        self.tracker=AcceptanceTracker(self.scenario)
        self.path=self.get_parameter('output_jsonl').value
        self.timeout_s=float(self.get_parameter('timeout_s').value)
        self.input_start_ns=int(self.get_parameter('input_start_ns').value)
        self.drive_nomotion_updates=bool(self.get_parameter('drive_nomotion_updates').value)
        self.timeout_reported=False
        self.map=None;self.pose=None;self.pose_stamp=None;self.particles=None;self.particle_stamp=None
        self.filter_updates={};self.nomotion_requests=0
        self.pending_particles=deque();self.pending_poses=deque()
        self.counts={'scan_received':0,'particle_cloud_received':0,'pose_updates':0,
                     'paired_filter_updates':0,'scored':0,'scan_update_missing':0,
                     'nomotion_requests':0}
        self.started=None;self.timing_origin=None;self.acceptance=None;self.pending_scans=deque();self.latest_fit_stamp=None
        self.fit_tracker=AcceptanceTracker(self.scenario);self.check_stamps=[];self.last_gate_pass=False
        map_qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid,'/map',self.on_map,map_qos)
        # At 10k+ particles the Python scoring callback can lag behind the
        # publisher. Retain the entire short S2 sequence (<=50 scans here).
        particle_qos=QoSProfile(depth=64,reliability=ReliabilityPolicy.BEST_EFFORT)
        pose_qos=QoSProfile(depth=1000,reliability=ReliabilityPolicy.RELIABLE)
        scan_qos=QoSProfile(depth=1000,reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(ParticleCloud,'/particle_cloud',self.on_particles,particle_qos)
        self.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',self.on_pose,pose_qos)
        self.update_client=self.create_client(Empty,'/request_nomotion_update')
        self.create_subscription(LaserScan,'/scan',lambda m:self.on_scan(m,False),scan_qos)
        self.create_subscription(LaserScan,'/check_scan',lambda m:self.on_scan(m,True),scan_qos)
        self.create_subscription(Imu,'/livox/imu',self.on_imu,qos_profile_sensor_data)
        self.create_timer(.02,self.drain_scans)
        self.create_timer(.1,self.check_deadline)
        self.create_timer(1.0,self.report_counts)

    def on_map(self,m):
        o=m.info.origin.position;q=m.info.origin.orientation
        self.map=Occupancy(np.asarray(m.data,dtype=np.int16).reshape((m.info.height,m.info.width)),
                           m.info.resolution,o.x,o.y,_yaw(q),65)

    def on_particles(self,m):
        particles=[(p.pose.position.x,p.pose.position.y,_yaw(p.pose.orientation),p.weight) for p in m.particles]
        cloud_stamp=m.header.stamp.sec*1_000_000_000+m.header.stamp.nanosec
        self.pending_particles.append((time.monotonic_ns(),particles,cloud_stamp))
        while len(self.pending_particles)>64:self.pending_particles.popleft()
        self.counts['particle_cloud_received']+=1
        self.pair_filter_update()
        # Nav2 1.3.13 publishes this cloud after processing a scan. Request the
        # next stationary update here, after the current update has completed;
        # requesting on /scan races the AMCL scan callback and can coalesce its
        # force_update_ flag with the scan already in flight.
        if (self.drive_nomotion_updates and self.scenario=='S2'
                and self.update_client.service_is_ready()):
            self.update_client.call_async(Empty.Request())
            self.nomotion_requests+=1
            self.counts['nomotion_requests']=self.nomotion_requests

    def on_pose(self,m):
        p=m.pose.pose.position;q=m.pose.pose.orientation
        pose=(p.x,p.y,_yaw(q))
        pose_stamp=m.header.stamp.sec*1_000_000_000+m.header.stamp.nanosec
        self.pending_poses.append((time.monotonic_ns(),pose,pose_stamp))
        while len(self.pending_poses)>64:self.pending_poses.popleft()
        self.counts['pose_updates']+=1
        self.pair_filter_update()

    def pair_filter_update(self):
        # The pose header is the source scan stamp. ParticleCloud uses node
        # time, so headers cannot be equated; pair one-to-one by local receive
        # time. AMCL emits the cloud before the pose within an update, but it
        # can emit a pose with no cloud on the first forced update. Never pair
        # a later cloud with that orphan pose. Cross-topic receive reordering
        # remains unscorable rather than silently shifting scan identity.
        if not self.pending_particles or not self.pending_poses:return
        pair=causal_cloud_pose_pair(self.pending_particles,self.pending_poses)
        if pair is None:return
        pi,qi=pair
        _,particles,cloud_stamp=self.pending_particles[pi]
        _,pose,pose_stamp=self.pending_poses[qi]
        del self.pending_particles[pi];del self.pending_poses[qi]
        self.filter_updates[pose_stamp]={'pose':pose,'particles':particles,
                                         'particle_cloud_stamp_ns':cloud_stamp}
        self.counts['paired_filter_updates']+=1
        if len(self.filter_updates)>1024:
            for old in sorted(self.filter_updates)[:-512]:self.filter_updates.pop(old,None)

    def on_imu(self,m):
        # Start the 60 s clock with the first conditioning IMU sample, not the
        # first scan after the two-second prefix. This is executor timing only;
        # AMCL still receives no private reference or true pose.
        stamp=m.header.stamp.sec*1_000_000_000+m.header.stamp.nanosec
        if (self.started is None and self.input_start_ns>0
                and self.input_start_ns-2_000_000_000<=stamp<self.input_start_ns):
            self.started=time.monotonic()
            self.timing_origin={'kind':'conditioning_imu','sensor_stamp_ns':stamp}

    def on_scan(self,m,is_check):
        now=time.monotonic()
        if self.started is None:
            self.started=now
            self.timing_origin={'kind':'first_scan_fallback','sensor_stamp_ns':
                m.header.stamp.sec*1_000_000_000+m.header.stamp.nanosec}
        self.counts['scan_received']+=1
        stamp=m.header.stamp.sec*1_000_000_000+m.header.stamp.nanosec
        # S2 confirmation uses distinct, post-update batches from the same
        # 3 s stream. The protocol does not prescribe a withheld S2 partition.
        # S1 continues to use its explicit same-frame 70/30 check scan.
        if self.scenario=='S2' and not is_check:is_check=True
        if not is_check:self.latest_fit_stamp=stamp
        elif self.scenario=='S2':self.latest_fit_stamp=stamp
        self.pending_scans.append((m,is_check,stamp,now-self.started,now))

    def drain_scans(self):
        # AMCL and this observer receive the same scan independently. Wait for
        # pose and particle messages stamped at/after that scan before scoring,
        # so the diagnostic cannot accidentally reuse the previous update.
        while self.pending_scans:
            m,is_check,stamp,elapsed,received=self.pending_scans[0]
            update=self.filter_updates.get(stamp)
            if self.map is None:
                break
            if update is None:
                if scan_update_state(stamp,self.filter_updates,time.monotonic()-received,
                                     self.timeout_s)=='wait':
                    break
                self.pending_scans.popleft();self.counts['scan_update_missing']+=1
                record={'sensor_stamp_ns':stamp,'input_kind':'check' if is_check else 'fit',
                        'terminal_event':'no_scan_aligned_amcl_update','wall_elapsed_s':elapsed,
                        'nomotion_requests':self.nomotion_requests}
                with open(self.path,'a',encoding='utf-8') as f:
                    f.write(json.dumps(record,sort_keys=True,allow_nan=False)+'\n')
                continue
            self.pending_scans.popleft()
            self.score_scan(m,is_check,stamp,elapsed,update)

    def score_scan(self,m,is_check,stamp,elapsed,update):
        now=time.monotonic()
        record={'sensor_stamp_ns':stamp,'wall_elapsed_s':elapsed,
                'finite_beams':int(np.isfinite(np.asarray(m.ranges,dtype=float)).sum()),
                'input_kind':'check' if is_check else 'fit',
                'have_map':self.map is not None,'have_pose':True,
                'pose_stamp_ns':stamp,'particle_stamp_ns':update['particle_cloud_stamp_ns'],
                'particle_count':len(update['particles']),
                'nomotion_requests':self.nomotion_requests}
        particles=update['particles'];pose=update['pose']
        modes=particle_modes(particles,5) if particles else []
        # Use the strongest AMCL particle mode as the live hypothesis. The
        # lower-rate /amcl_pose publication is logged separately and is not
        # copied forward as if it were a fresh scan-aligned estimate.
        mode_pose=(modes[0].x,modes[0].y,modes[0].yaw) if modes else None
        record['amcl_pose']=pose
        record['amcl_pose_stamp_ns']=stamp
        if self.map is not None and mode_pose is not None and particles:
            scan=Scan(np.asarray(m.ranges,dtype=float),m.angle_min,m.angle_increment,m.range_min,m.range_max)
            result=evaluate(scan,particles,mode_pose,self.map)
            record.update({'gate_passes':result.passes,'gate_reason':result.reason,
                           'particle_concentration':result.particle_concentration,
                           'primary_support':result.primary_support,'runner_up_support':result.runner_up_support,
                           'beam_span_rad':result.beam_span_rad,'pose':mode_pose})
            self.last_gate_pass=result.passes
            if is_check:
                if result.passes:self.check_stamps.append(stamp)
                else:self.check_stamps=[]
            # In S1 the fit and withheld check partitions share one stamp.
            # Only the check partition may cause a commit-to-pose event.
            event=self.fit_tracker.observe(stamp,elapsed,mode_pose,result) if is_check else None
            confirmed=(self.scenario=='S1' or
                (len(self.check_stamps)>=2 and self.check_stamps[-1]-self.check_stamps[0]>=500_000_000))
            if event and is_check and confirmed:
                self.acceptance={'kind':'accept','sensor_stamp_ns':stamp,
                                 'wall_elapsed_s':elapsed,'pose':mode_pose,
                                 'confirmation_count':len(self.check_stamps)}
                record['acceptance_event']=self.acceptance
        with open(self.path,'a',encoding='utf-8') as f:
            f.write(json.dumps(record,sort_keys=True,allow_nan=False)+'\n')
        self.counts['scored']+=1
        if self.acceptance:
            self.get_logger().info('observable acceptance gate passed')

    def check_deadline(self):
        if self.started is None or self.acceptance is not None or self.timeout_reported:
            return
        elapsed=time.monotonic()-self.started
        if elapsed<self.timeout_s:return
        event={'terminal_event':'timeout','wall_elapsed_s':elapsed,'timeout_s':self.timeout_s}
        event['timing_origin']=self.timing_origin
        event['observer_counts']=dict(self.counts)
        event['pending_scans']=len(self.pending_scans)
        event['unmatched_particle_clouds']=len(self.pending_particles)
        event['unmatched_pose_updates']=len(self.pending_poses)
        with open(self.path,'a',encoding='utf-8') as f:
            f.write(json.dumps(event,sort_keys=True,allow_nan=False)+'\n')
        self.timeout_reported=True
        self.get_logger().info('observable acceptance deadline reached')

    def report_counts(self):
        self.get_logger().info('observer diagnostics '+json.dumps(self.counts,sort_keys=True))


def main():
    rclpy.init();node=Observer()
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
