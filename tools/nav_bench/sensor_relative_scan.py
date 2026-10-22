#!/usr/bin/env python3
"""Experimental gravity-levelled /scan from Livox CustomMsg without absolute height.

The vertical band is specified relative to the IMU/LiDAR origin via a candidate
sensor_height. It is intended for stationary S1/S2 development only. This does
not create a physical base_footprint TF or certify compatibility with the PGM.
"""
import json
import math
import time
from copy import deepcopy

import numpy as np


def custom_point_bounds_ns(timebase_ns,offsets_ns):
    """Return actual first/last point times of one Livox CustomMsg."""
    offsets=list(offsets_ns)
    if not offsets or timebase_ns<0 or any(type(x) is not int or x<0 for x in offsets):
        raise ValueError('invalid CustomMsg point timing')
    first=int(timebase_ns)+min(offsets)
    last=int(timebase_ns)+max(offsets)
    if last<first:raise ValueError('regressive CustomMsg point timing')
    return first,last


def complete_point_window(first_ns,last_ns,start_ns,end_ns):
    """Select only complete frames in the predeclared half-open exposure."""
    return start_ns<=first_ns<=last_ns<end_ns


def project_scan(points_xyz, gravity_up, sensor_height_m, *, angle_min=-math.pi,
                 angle_max=math.pi, angle_increment=math.radians(.5),
                 min_height=.10, max_height=1.80, range_min=.5, range_max=60.,
                 fov_deg=None, fov_center_deg=0., excluded_angle_sectors_deg=(),
                 sensor_relative_band_m=None):
    """Project a cloud into a gravity-levelled scan under explicit geometry settings.

    A sensor-relative band bypasses floor-height translation. Otherwise the
    floor-relative ``min_height/max_height`` are shifted by sensor height.
    """
    p=np.asarray(points_xyz,dtype=float); g=np.asarray(gravity_up,dtype=float)
    if p.ndim!=2 or p.shape[1]!=3 or not np.isfinite(p).all():
        raise ValueError('points_xyz must be finite Nx3')
    if g.shape!=(3,) or not np.isfinite(g).all() or np.linalg.norm(g)<1e-8:
        raise ValueError('gravity_up must be finite and nonzero')
    if (not (0<angle_increment) or not (0<=range_min<range_max)
            or not np.isfinite([sensor_height_m,min_height,max_height,fov_center_deg,
                                angle_increment,range_min,range_max]).all()):
        raise ValueError('invalid projection parameters')
    if sensor_relative_band_m is None:
        if not (0<=min_height<max_height and sensor_height_m>0):
            raise ValueError('invalid floor-relative band or sensor height')
        z_min=min_height-sensor_height_m;z_max=max_height-sensor_height_m
    else:
        z_min,z_max=map(float,sensor_relative_band_m)
        if not np.isfinite([z_min,z_max]).all() or z_min>=z_max:
            raise ValueError('invalid sensor-relative band')
    if fov_deg is None:
        fov_deg=math.degrees(angle_max-angle_min)
        fov_center_deg=math.degrees((angle_min+angle_max)/2.)
    if not (0<fov_deg<=360):
        raise ValueError('fov_deg must be in (0, 360]')
    n_float=math.radians(fov_deg)/angle_increment
    n=int(round(n_float))
    if n<1 or not math.isclose(n_float,n,rel_tol=0.,abs_tol=1e-6):
        raise ValueError('FOV must be an integer multiple of angle increment')
    angle_min=math.radians(fov_center_deg-fov_deg/2.)
    angle_max=angle_min+(n-1)*angle_increment
    up=g/np.linalg.norm(g)
    ref=np.array([1.,0.,0.]) if abs(up[0])<.9 else np.array([0.,1.,0.])
    x=ref-up*np.dot(ref,up); x/=np.linalg.norm(x)
    y=np.cross(up,x)
    z=p@up
    keep=(z>=z_min)&(z<=z_max)
    xx=p@x; yy=p@y; rr=np.hypot(xx,yy)
    keep &= (rr>=range_min)&(rr<=range_max)
    angle=np.arctan2(yy,xx)
    delta=(angle-math.radians(fov_center_deg)+math.pi)%(2*math.pi)-math.pi
    if fov_deg<360.:
        keep &= np.abs(delta)<=math.radians(fov_deg)/2.+1e-12
    for lo_deg,hi_deg in excluded_angle_sectors_deg:
        lo=float(lo_deg);hi=float(hi_deg)
        if not (-180<=lo<hi<=180):
            raise ValueError('masked sector bounds must satisfy -180 <= lo < hi <= 180')
        keep &= ~((np.degrees(delta)>=lo)&(np.degrees(delta)<hi))
    ranges=np.full(n,np.inf,dtype=float)
    if keep.any():
        bins=np.floor((np.degrees(delta[keep])+fov_deg/2.)/
                      math.degrees(angle_increment)).astype(int)
        if fov_deg==360.:
            bins %= n
        valid=(bins>=0)&(bins<n)
        np.minimum.at(ranges,bins[valid],rr[keep][valid])
    return {'ranges':ranges,'finite_count':int(np.isfinite(ranges).sum()),
            'beam_count':n,'angle_min':float(angle_min),'angle_max':float(angle_max),
            'angle_increment':float(angle_increment),'range_min':float(range_min),
            'range_max':float(range_max),'sensor_height_hypothesis_m':float(sensor_height_m),
            'point_count':int(len(p)),'band_point_count':int(keep.sum()),
            'fov_deg':float(fov_deg),'fov_center_deg':float(fov_center_deg),
            'sensor_relative_band_m':list(sensor_relative_band_m) if sensor_relative_band_m is not None else None}


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
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Imu, LaserScan
    sensor_qos=QoSProfile(depth=1000,reliability=ReliabilityPolicy.BEST_EFFORT)
    from collections import deque
    from threading import Lock

    class ScanNode(Node):
        def __init__(self):
            super().__init__('sensor_relative_scan')
            self.scenario=str(self.declare_parameter('scenario','S2').value)
            if self.scenario not in ('S1','S2'):
                raise ValueError('stationary scan frontend supports only S1/S2')
            self.h=self.declare_parameter('sensor_height_hypothesis_m',1.0).value
            self.min_h=self.declare_parameter('min_obstacle_height',.10).value
            self.max_h=self.declare_parameter('max_obstacle_height',1.80).value
            self.fov_deg=float(self.declare_parameter('fov_deg',360.).value)
            self.fov_center_deg=float(self.declare_parameter('fov_center_deg',0.).value)
            self.angle_increment_deg=float(self.declare_parameter('angle_increment_deg',.5).value)
            self.range_min=float(self.declare_parameter('range_min',.5).value)
            self.range_max=float(self.declare_parameter('range_max',60.).value)
            output_topic=str(self.declare_parameter('output_topic','/scan').value)
            if output_topic not in ('/scan','/candidate_scan'):
                raise ValueError('output_topic must be /scan or /candidate_scan')
            reliability=str(self.declare_parameter('input_reliability','best_effort').value).lower()
            if reliability not in ('best_effort','reliable'):
                raise ValueError('input_reliability must be best_effort or reliable')
            input_qos=QoSProfile(depth=1000,reliability=(ReliabilityPolicy.RELIABLE
                if reliability=='reliable' else ReliabilityPolicy.BEST_EFFORT))
            self.excluded_sectors=json.loads(str(self.declare_parameter('excluded_angle_sectors_deg','[]').value))
            relative_band_raw=str(self.declare_parameter('sensor_relative_band_m','none').value)
            relative_band=[] if relative_band_raw.lower() in ('','none') else json.loads(relative_band_raw)
            self.sensor_relative_band=tuple(relative_band) if relative_band else None
            self.input_start_ns=self.declare_parameter('input_start_ns',0).value
            self.input_end_ns=self.declare_parameter('input_end_ns',9223372036854775807).value
            self.fit_end_ns=self.declare_parameter('fit_end_ns',9223372036854775807).value
            self.imu_median_ns=self.declare_parameter('imu_median_ns',0.0).value
            self.guard=Lock()
            self.imu_group=MutuallyExclusiveCallbackGroup()
            self.lidar_group=MutuallyExclusiveCallbackGroup()
            self.imu=deque(maxlen=1000)
            self.pending_lidar=deque(maxlen=128)
            self.conditioning_start_ns=self.input_start_ns-2_000_000_000
            self.gravity=None;self.gravity_info=None;self.conditioning_finalized=False;self.draining=False
            self.counts={'imu':0,'lidar':0,'outside_window':0,'insufficient_imu':0,'projection_error':0,
                         'fit_published':0,'check_published':0,'last_lidar_stamp_ns':None,
                         'first_lidar_stamp_ns':None,'last_imu_stamp_ns':None,
                         'first_point_stamp_ns':None,'last_point_stamp_ns':None,
                         'conditioning_queued_lidar':0,'window_lidar_received':0,
                         'excess_s1_frames':0,
                         'processing_ms_total':0.,'processing_ms_max':0.}
            scan_qos=QoSProfile(depth=100,reliability=ReliabilityPolicy.RELIABLE)
            self.pub_fit=self.create_publisher(LaserScan,output_topic,scan_qos)
            self.pub_check=self.create_publisher(LaserScan,'/check_scan',sensor_qos)
            # rosbag2 can publish short bursts when simulated time advances;
            # retain the complete selected window rather than the default 5 samples.
            self.create_subscription(Imu,'/livox/imu',self.on_imu,input_qos,
                                     callback_group=self.imu_group)
            self.create_subscription(CustomMsg,'/livox/lidar',self.on_lidar,input_qos,
                                     callback_group=self.lidar_group)
            self.create_timer(1.0,self.report_counts)

        def on_imu(self,m):
            t=m.header.stamp.sec*1_000_000_000+m.header.stamp.nanosec
            a=m.linear_acceleration
            with self.guard:
                self.imu.append((t,np.array([a.x,a.y,a.z],dtype=float)))
                self.counts['imu']+=1;self.counts['last_imu_stamp_ns']=t
                if self.conditioning_finalized or t<self.input_start_ns:return
                self.gravity,self.gravity_info=conditioning_gravity(
                    self.imu,self.conditioning_start_ns,self.input_start_ns,
                    self.imu_median_ns if self.imu_median_ns>0 else None)
                self.conditioning_finalized=True;self.draining=True
            # Keep later LiDAR callbacks responsive while flushing any frames
            # that arrived before the IMU prefix completed. They append under
            # the same lock and are drained in point-time order.
            while True:
                with self.guard:
                    queued=sorted(self.pending_lidar,key=lambda item:item[1])
                    self.pending_lidar.clear()
                    if not queued:
                        self.draining=False
                        return
                    if self.gravity is None:
                        self.counts['insufficient_imu']+=len(queued)
                if self.gravity is not None:
                    for cloud,last_ns in queued:self.process_lidar(cloud,last_ns)

        def on_lidar(self,m):
            t=m.header.stamp.sec*1_000_000_000+m.header.stamp.nanosec
            try:
                first_ns,last_ns=custom_point_bounds_ns(int(m.timebase),
                    (int(point.offset_time) for point in m.points))
            except ValueError:
                with self.guard:self.counts['projection_error']+=1
                return
            with self.guard:
                self.counts['lidar']+=1
                if self.counts['first_lidar_stamp_ns'] is None:self.counts['first_lidar_stamp_ns']=t
                self.counts['last_lidar_stamp_ns']=t
                self.counts['first_point_stamp_ns']=first_ns if self.counts['first_point_stamp_ns'] is None else self.counts['first_point_stamp_ns']
                self.counts['last_point_stamp_ns']=last_ns
                if not complete_point_window(first_ns,last_ns,self.input_start_ns,self.input_end_ns):
                    self.counts['outside_window']+=1
                    return
                self.counts['window_lidar_received']+=1
                if self.scenario=='S1' and self.counts['window_lidar_received']>1:
                    self.counts['excess_s1_frames']+=1
                    return
                if not self.conditioning_finalized or self.draining:
                    if len(self.pending_lidar)==self.pending_lidar.maxlen:
                        self.counts['insufficient_imu']+=1
                        return
                    self.pending_lidar.append((m,last_ns))
                    self.counts['conditioning_queued_lidar']+=1
                    return
                if self.gravity is None:
                    self.counts['insufficient_imu']+=1
                    return
            self.process_lidar(m,last_ns)

        def process_lidar(self,m,last_ns):
            started=time.perf_counter()
            t=last_ns
            xyz=np.asarray([(p.x,p.y,p.z) for p in m.points],dtype=float)
            try:
                s=project_scan(xyz,self.gravity,float(self.h),min_height=float(self.min_h),
                    max_height=float(self.max_h),range_min=self.range_min,range_max=self.range_max,
                    fov_deg=self.fov_deg,fov_center_deg=self.fov_center_deg,
                    angle_increment=math.radians(self.angle_increment_deg),
                    excluded_angle_sectors_deg=self.excluded_sectors,
                    sensor_relative_band_m=self.sensor_relative_band)
            except ValueError as e:
                self.counts['projection_error']+=1;self.get_logger().warning(str(e));return
            out=LaserScan();out.header.frame_id='livox_level'
            out.header.stamp.sec=last_ns//1_000_000_000
            out.header.stamp.nanosec=last_ns%1_000_000_000
            out.angle_min=s['angle_min'];out.angle_max=s['angle_max'];out.angle_increment=s['angle_increment']
            out.scan_time=.1;out.time_increment=0.;out.range_min=s['range_min'];out.range_max=s['range_max']
            out.ranges=s['ranges'].tolist();out.intensities=[]
            if self.scenario=='S1':
                from amcl_acceptance import Scan, split_s1_scan
                source=Scan(np.asarray(out.ranges,dtype=float),out.angle_min,
                            out.angle_increment,out.range_min,out.range_max)
                fit,check=split_s1_scan(source)
                check_out=deepcopy(out)
                out.ranges=fit.ranges.tolist()
                self.pub_fit.publish(out);self.counts['fit_published']+=1
                check_out.ranges=check.ranges.tolist()
                self.pub_check.publish(check_out);self.counts['check_published']+=1
            else:
                (self.pub_fit if t<self.fit_end_ns else self.pub_check).publish(out)
                self.counts['fit_published' if t<self.fit_end_ns else 'check_published']+=1
            duration_ms=(time.perf_counter()-started)*1000.
            self.counts['processing_ms_total']+=duration_ms
            self.counts['processing_ms_max']=max(self.counts['processing_ms_max'],duration_ms)

        def report_counts(self):
            state=dict(self.counts);state['conditioning']=self.gravity_info
            self.get_logger().info('scan diagnostics '+json.dumps(state,sort_keys=True))

    rclpy.init();node=ScanNode();executor=MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:executor.spin()
    finally:executor.shutdown();node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
