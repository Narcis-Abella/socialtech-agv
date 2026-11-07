"""Covered-lidar scenario of gn_supervisor in the full simulation (sup_sim.sh): start-up localization, a cover of the lidar, the uncover, the way back to MONITOR.
usage: sup_cover.py OUT.json [cover_s=20] [settle_s=10] [--push-after S] [--move-covered M]      sim time for the scenario, wall time next to it for the RTF.
  --push-after S    S seconds after the uncover the robot drives 0.4 m (a push while the relocalizer works): the pose of that attempt must be refused and a later one released
  --move-covered M  the robot drives M metres forward while the lidar is covered: GN dead-reckons on the old place, the relocalizer finds the new one and GN must confirm it
The headline is `uncover_to_monitor_s`: from the lidar seeing again to Nav2 released (MONITOR: confirmed, or verified_gn_agrees when GN already agrees). Pose error: GN's pose against the Gazebo pose, 3 s after the release."""
import json
import math
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Scenario(Node):
    def __init__(self):
        super().__init__('sup_cover', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.events, self.state, self.gt, self.gn = [], None, None, None
        self.create_subscription(String, 'gn_supervisor/state', self.on_state, LATCHED)
        self.create_subscription(String, 'gn_loc/status', lambda m: setattr(self, 'status', m.data), 10)
        self.create_subscription(Odometry, '/odom_gt_raw', lambda m: setattr(self, 'gt', m), QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(PoseWithCovarianceStamped, 'gn_loc/pose', lambda m: setattr(self, 'gn', m), LATCHED)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', lambda m: self.log('initialpose', x=m.pose.pose.position.x, y=m.pose.pose.position.y), 10)
        self.status = ''
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)

    def sim(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def log(self, what, **kw):
        self.events.append({'sim': round(self.sim(), 3), 'wall': round(time.time(), 3), 'what': what, **kw})
        print(self.events[-1], flush=True)

    def on_state(self, m):
        d = json.loads(m.data)
        self.state = (d['state'], d['reason'])
        info = d.get('last_reloc') or {}
        self.log('state', state=d['state'], reason=d['reason'], hold=d.get('hold'), why=info.get('why'), reloc_info=info.get('info'))

    def spin_until(self, cond, timeout_sim, what):
        t0 = self.sim()
        while rclpy.ok() and not cond():
            rclpy.spin_once(self, timeout_sec=.05)
            if self.sim() - t0 > timeout_sim:
                self.log('timeout', waiting_for=what)
                return False
        return True

    def spin_for(self, seconds):
        t0 = self.sim()
        while rclpy.ok() and self.sim() - t0 < seconds:
            rclpy.spin_once(self, timeout_sec=.05)

    def drive(self, metres, v=.3):
        """Forward `metres` at v (sim time), then stop."""
        t0, m = self.sim(), Twist()
        m.linear.x = v
        while rclpy.ok() and self.sim() - t0 < metres / v:
            self.cmd.publish(m)
            rclpy.spin_once(self, timeout_sec=.05)
        self.cmd.publish(Twist())
        self.log('drive', metres=metres)

    def cover(self, on):
        subprocess.run(['ros2', 'param', 'set', '/livox_mid360_sim', 'cover', 'true' if on else 'false'], check=True, capture_output=True)
        self.log('cover' if on else 'uncover')

    def pose_error(self):
        if not (self.gt and self.gn):
            return None
        g, p = self.gt.pose.pose, self.gn.pose.pose
        return {'xy_m': math.hypot(g.position.x - p.position.x, g.position.y - p.position.y), 'yaw_deg': abs(math.degrees(wrap(yaw_of(g.orientation) - yaw_of(p.orientation))))}


def first(events, after, pred):
    return next((e for e in events if e['sim'] >= after and e['what'] == 'state' and pred(e)), None)


def main():
    args = sys.argv[1:]
    opt = {}
    for flag in ('--push-after', '--move-covered'):
        if flag in args:
            i = args.index(flag); opt[flag] = float(args[i + 1]); del args[i:i + 2]
    out, cover_s, settle_s = args[0], float(args[1]) if len(args) > 1 else 20., float(args[2]) if len(args) > 2 else 10.
    rclpy.init()
    n = Scenario()
    s = {'options': opt}
    ok = n.spin_until(lambda: n.state and n.state[0] == 'MONITOR', 240., 'start-up localization (MONITOR)')
    s['startup_localized_at_sim_s'] = n.sim() if ok else None
    if ok:
        n.spin_for(settle_s)
        s['pose_error_before_cover'] = n.pose_error()
        t_cover = n.sim(); n.cover(True)
        if '--move-covered' in opt:
            n.spin_for(6.)                                               # long enough for the stop and the first looks of the relocalizer
            n.drive(opt['--move-covered'])
            n.spin_for(max(cover_s - 6. - opt['--move-covered'] / .3, 1.))
        else:
            n.spin_for(cover_s)
        t_uncover = n.sim(); n.cover(False)
        if '--push-after' in opt:
            n.spin_for(opt['--push-after']); n.drive(.4)
        back = lambda: next((e for e in reversed(n.events) if e['what'] == 'state' and e['sim'] > t_uncover and e['state'] == 'MONITOR'), None)
        done = n.spin_until(lambda: back() is not None, 240., 'MONITOR after the uncover')
        t_done = back()['sim'] if done else n.sim()
        stop = first(n.events, t_cover, lambda e: e['state'] != 'MONITOR')
        s.update({'cover_s': cover_s, 'stopped_after_cover_s': stop['sim'] - t_cover if stop else None, 'stop_reason': stop['reason'] if stop else None,
                  'back_to_monitor': done, 'monitor_reason': back()['reason'] if done else None,
                  'uncover_to_monitor_s': t_done - t_uncover if done else None,
                  'relocalizations': sum(1 for e in n.events if e['what'] == 'state' and e['state'] == 'RELOCALIZING' and e['sim'] > t_cover),
                  'refused': [e['why'] for e in n.events if e['what'] == 'state' and e['sim'] > t_cover and (e.get('why') or '').startswith('moved')]})
        n.spin_for(3.)
        s['pose_error_after'] = n.pose_error()
    s['events'] = n.events
    json.dump(s, open(out, 'w'), indent=1)
    print(json.dumps({k: v for k, v in s.items() if k != 'events'}, indent=1))
    rclpy.shutdown()


if __name__ == '__main__':
    main()
