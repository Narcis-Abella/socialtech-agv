#!/usr/bin/env python3
"""Experimental gravity-levelled /scan from Livox CustomMsg without absolute height.

The vertical band is specified relative to the IMU/LiDAR origin via a candidate
sensor_height. It is intended for stationary S1/S2 development only. This does
not create a physical base_footprint TF or certify compatibility with the PGM.
"""
import math

import numpy as np


def project_scan(points_xyz, gravity_up, sensor_height_m, *, angle_min=-math.pi,
                 angle_max=math.pi, angle_increment=math.radians(.5),
                 min_height=.10, max_height=1.80, range_min=.5, range_max=60.):
    """Project one raw cloud into a level sensor-origin scan for one height hypothesis."""
    p=np.asarray(points_xyz,dtype=float); g=np.asarray(gravity_up,dtype=float)
    if p.ndim!=2 or p.shape[1]!=3 or not np.isfinite(p).all():
        raise ValueError('points_xyz must be finite Nx3')
    if g.shape!=(3,) or not np.isfinite(g).all() or np.linalg.norm(g)<1e-8:
        raise ValueError('gravity_up must be finite and nonzero')
    if not (0<=min_height<max_height and sensor_height_m>0 and 0<angle_increment):
        raise ValueError('invalid height/angle parameters')
    up=g/np.linalg.norm(g)
    ref=np.array([1.,0.,0.]) if abs(up[0])<.9 else np.array([0.,1.,0.])
    x=ref-up*np.dot(ref,up); x/=np.linalg.norm(x)
    y=np.cross(up,x)
    z=p@up
    # Map obstacle band [min_height,max_height] above floor to sensor-relative z.
    keep=(z>=min_height-sensor_height_m)&(z<=max_height-sensor_height_m)
    xx=p@x; yy=p@y; rr=np.hypot(xx,yy)
    keep &= (rr>=range_min)&(rr<=range_max)
    n=int(math.ceil((angle_max-angle_min)/angle_increment))
    ranges=np.full(n,np.inf,dtype=float)
    if keep.any():
        a=np.arctan2(yy[keep],xx[keep])
        bins=np.floor((a-angle_min)/angle_increment).astype(int)
        valid=(bins>=0)&(bins<n)
        np.minimum.at(ranges,bins[valid],rr[keep][valid])
    return {'ranges':ranges,'finite_count':int(np.isfinite(ranges).sum()),
            'beam_count':n,'angle_min':float(angle_min),'angle_max':float(angle_max),
            'angle_increment':float(angle_increment),'range_min':float(range_min),
            'range_max':float(range_max),'sensor_height_hypothesis_m':float(sensor_height_m),
            'point_count':int(len(p)),'band_point_count':int(keep.sum())}


def conditioning_gravity(samples, start_ns, end_ns, dataset_median_ns=None):
    """Robustly freeze gravity from a sensor-only, two-second IMU prefix.

    ``samples`` is an ordered iterable of (header_stamp_ns, acceleration_xyz).
    The existing protocol's 4x hard-gap and 95% coverage rules are applied;
    no truth or map data is accepted by this function.
    """
    rows=[(int(t),np.asarray(a,dtype=float)) for t,a in samples if start_ns<=int(t)<end_ns]
    if len(rows)<2:
        return None,{'reason':'too_few_conditioning_samples','observed':len(rows)}
    times=np.asarray([t for t,_ in rows],dtype=np.int64)
    dt=np.diff(times)
    if np.any(dt<=0):
        return None,{'reason':'non_monotonic_imu','observed':len(rows)}
    # The protocol defines cadence once over the whole dataset. A short replay
    # prefix must not substitute its own local median, especially after DDS loss.
    median_ns=float(dataset_median_ns) if dataset_median_ns is not None else float(np.median(dt))
    if median_ns<=0:
        return None,{'reason':'invalid_imu_cadence','observed':len(rows)}
    expected=max(1,int(round((end_ns-start_ns)/median_ns)))
    coverage=len(rows)/expected
    if np.any(dt>4.*median_ns):
        return None,{'reason':'imu_hard_gap','observed':len(rows),'expected':expected,
                     'coverage_ratio':coverage,'median_dt_ns':median_ns}
    if coverage<.95:
        return None,{'reason':'imu_coverage_below_95_percent','observed':len(rows),
                     'expected':expected,'coverage_ratio':coverage,'median_dt_ns':median_ns}
    gravity=np.median(np.stack([a for _,a in rows]),axis=0)
    norm=float(np.linalg.norm(gravity))
    if not np.isfinite(gravity).all() or norm<1e-8:
        return None,{'reason':'invalid_gravity_vector','observed':len(rows),
                     'expected':expected,'coverage_ratio':coverage,'median_dt_ns':median_ns}
    return gravity,{'reason':'ok','observed':len(rows),'expected':expected,
                    'coverage_ratio':coverage,'median_dt_ns':median_ns}


def main():
    import json
    import rclpy
    from livox_ros_driver2.msg import CustomMsg
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Imu, LaserScan
    sensor_qos=QoSProfile(depth=1000,reliability=ReliabilityPolicy.BEST_EFFORT)
    from collections import deque

    class ScanNode(Node):
        def __init__(self):
            super().__init__('sensor_relative_scan')
            self.h=self.declare_parameter('sensor_height_hypothesis_m',1.0).value
            self.min_h=self.declare_parameter('min_obstacle_height',.10).value
            self.max_h=self.declare_parameter('max_obstacle_height',1.80).value
            self.input_start_ns=self.declare_parameter('input_start_ns',0).value
            self.input_end_ns=self.declare_parameter('input_end_ns',9223372036854775807).value
            self.fit_end_ns=self.declare_parameter('fit_end_ns',9223372036854775807).value
            self.imu_median_ns=self.declare_parameter('imu_median_ns',0.0).value
            self.imu=deque(maxlen=1000)
            self.conditioning_start_ns=self.input_start_ns-2_000_000_000
            self.gravity=None;self.gravity_info=None
            self.counts={'imu':0,'lidar':0,'outside_window':0,'insufficient_imu':0,'projection_error':0,
                         'fit_published':0,'check_published':0,'last_lidar_stamp_ns':None,
                         'first_lidar_stamp_ns':None,'last_imu_stamp_ns':None}
            self.pub_fit=self.create_publisher(LaserScan,'/scan',sensor_qos)
            self.pub_check=self.create_publisher(LaserScan,'/check_scan',sensor_qos)
            # rosbag2 can publish short bursts when simulated time advances;
            # retain the complete selected window rather than the default 5 samples.
            self.create_subscription(Imu,'/livox/imu',self.on_imu,sensor_qos)
            self.create_subscription(CustomMsg,'/livox/lidar',self.on_lidar,sensor_qos)
            self.create_timer(1.0,self.report_counts)

        def on_imu(self,m):
            t=m.header.stamp.sec*1_000_000_000+m.header.stamp.nanosec
            a=m.linear_acceleration
            self.imu.append((t,np.array([a.x,a.y,a.z],dtype=float)))
            self.counts['imu']+=1;self.counts['last_imu_stamp_ns']=t

        def on_lidar(self,m):
            t=m.header.stamp.sec*1_000_000_000+m.header.stamp.nanosec
            self.counts['lidar']+=1
            if self.counts['first_lidar_stamp_ns'] is None:self.counts['first_lidar_stamp_ns']=t
            self.counts['last_lidar_stamp_ns']=t
            if not self.input_start_ns<=t<self.input_end_ns:
                self.counts['outside_window']+=1
                return
            if self.gravity is None:
                self.gravity,self.gravity_info=conditioning_gravity(
                    self.imu,self.conditioning_start_ns,self.input_start_ns,
                    self.imu_median_ns if self.imu_median_ns>0 else None)
            if self.gravity is None:
                self.counts['insufficient_imu']+=1
                return
            xyz=np.asarray([(p.x,p.y,p.z) for p in m.points],dtype=float)
            try:
                s=project_scan(xyz,self.gravity,float(self.h),min_height=float(self.min_h),max_height=float(self.max_h))
            except ValueError as e:
                self.counts['projection_error']+=1;self.get_logger().warning(str(e));return
            out=LaserScan();out.header=m.header;out.header.frame_id='livox_level'
            out.angle_min=s['angle_min'];out.angle_max=s['angle_max'];out.angle_increment=s['angle_increment']
            out.scan_time=.1;out.time_increment=0.;out.range_min=s['range_min'];out.range_max=s['range_max']
            out.ranges=s['ranges'].tolist();out.intensities=[]
            (self.pub_fit if t<self.fit_end_ns else self.pub_check).publish(out)
            self.counts['fit_published' if t<self.fit_end_ns else 'check_published']+=1

        def report_counts(self):
            state=dict(self.counts);state['conditioning']=self.gravity_info
            self.get_logger().info('scan diagnostics '+json.dumps(state,sort_keys=True))

    rclpy.init();rclpy.spin(ScanNode());rclpy.shutdown()


if __name__=='__main__':main()
