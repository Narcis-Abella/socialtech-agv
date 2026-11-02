"""One look at what the robot's Nav2 sees around it, in the full-chain simulation (nav_full_sim.sh DIAG=1): the /scan returns nearer than 1.2 m, the lethal cells of the local costmap within 1.2 m
of the robot, and the pose in map. usage: nav_diag.py (sim time, Nav2 running)"""
import math
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


def main():
    rclpy.init()
    n = Node('nav_diag', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
    buf = Buffer(); TransformListener(buf, n)
    got = {}
    n.create_subscription(LaserScan, '/scan', lambda m: got.__setitem__('scan', m), qos_profile_sensor_data)
    n.create_subscription(OccupancyGrid, '/local_costmap/costmap', lambda m: got.__setitem__('cm', m), QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
    t0 = time.time()
    while rclpy.ok() and ('scan' not in got or 'cm' not in got) and time.time() - t0 < 60:
        rclpy.spin_once(n, timeout_sec=.2)
    t1 = time.time()
    while rclpy.ok() and time.time() - t1 < 4:           # let the TF listener fill its buffer
        rclpy.spin_once(n, timeout_sec=.2)
    s, cm = got['scan'], got['cm']
    r = np.array(s.ranges); ang = s.angle_min + s.angle_increment * np.arange(len(r))
    near = np.flatnonzero(np.isfinite(r) & (r < 1.2))
    print(f'scan frame {s.header.frame_id}: {len(r)} beams, {int(np.isfinite(r).sum())} finite; nearer than 1.2 m: {len(near)}')
    for i in near[:40]:
        print(f'  beam {i}: angle {math.degrees(ang[i]):7.1f} deg  range {r[i]:.2f} m')
    for frame in ('map', 'odom'):
        try:
            tf = buf.lookup_transform(frame, 'base_footprint', rclpy.time.Time()); p = tf.transform.translation
            print(f'base_footprint in {frame}: {p.x:.2f} {p.y:.2f} {p.z:.2f}')
        except Exception as e:
            print(frame, 'tf error', e)
    info = cm.info; d = np.array(cm.data, int).reshape(info.height, info.width)
    tf = buf.lookup_transform(cm.header.frame_id, 'base_footprint', rclpy.time.Time()); rx, ry = tf.transform.translation.x, tf.transform.translation.y
    jj, ii = np.nonzero(d >= 253)
    x, y = info.origin.position.x + (ii + .5) * info.resolution, info.origin.position.y + (jj + .5) * info.resolution
    dist = np.hypot(x - rx, y - ry)
    m = dist < 1.2
    print(f'local costmap ({cm.header.frame_id}, {info.width}x{info.height} at {info.resolution} m): {len(jj)} lethal cells, {int(m.sum())} within 1.2 m of the robot at ({rx:.2f}, {ry:.2f})')
    ci, cj = np.meshgrid(np.arange(info.width), np.arange(info.height))
    cx, cy = info.origin.position.x + (ci + .5) * info.resolution, info.origin.position.y + (cj + .5) * info.resolution
    near_robot = np.hypot(cx - rx, cy - ry) < .6
    vals, cnt = np.unique(d[near_robot], return_counts=True)
    print('costs within 0.6 m of the robot (value: cells):', dict(zip(vals.tolist(), cnt.tolist())))
    print('whole costmap:', dict(zip(*[x.tolist() for x in np.unique(d, return_counts=True)])))
    import subprocess
    for prm in ('FollowPath.use_collision_detection', 'FollowPath.max_allowed_time_to_collision_up_to_carrot', 'FollowPath.allow_reversing', 'FollowPath.lookahead_dist'):
        print(prm, subprocess.run(['ros2', 'param', 'get', '/controller_server', prm], capture_output=True, text=True).stdout.strip())
    for k in np.argsort(dist)[:25]:
        if m[k]:
            print(f'  lethal at ({x[k]:.2f}, {y[k]:.2f}) dist {dist[k]:.2f} bearing {math.degrees(math.atan2(y[k] - ry, x[k] - rx)):.0f} deg')


if __name__ == '__main__':
    main()
