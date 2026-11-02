"""Does a relocalizer search disturb FAST-LIO? Latency and gaps of /Odometry while the search worker runs, against the idle stack (lat_sim.sh).
usage: lat_ab.py OUT.json INPUT_DIR [reps=4] [idle_s=30] [--z-band LO HI]   INPUT_DIR: a prepared search input (map/fit/check.xyz, manifest.json) such as runs/gn_supervisor/reloc*/input
Modes: idle (no search), search (as gn_supervisor starts it), nice (the same under `nice -n 10`). Latency = sim clock at arrival - header stamp of the odometry (it includes the 0.1 s scan period);
a gap is the sim time between two consecutive messages (nominal 0.1 s). The simulator runs at ~0.95 real time, so all numbers are in sim seconds."""
import json
import subprocess
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
BEST_EFFORT = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)


def pct(xs, q):
    xs = sorted(xs)
    return xs[min(int(q * len(xs)), len(xs) - 1)] if xs else float('nan')


def summarize(lat, gaps):
    ms = lambda xs, q: round(1000 * pct(xs, q), 1)
    return {'n': len(lat), 'lat_ms': [ms(lat, .5), ms(lat, .95), ms(lat, 1.)], 'gap_ms': [ms(gaps, .5), ms(gaps, .95), ms(gaps, 1.)], 'gaps_over_150ms': sum(g > .15 for g in gaps)}


class Probe(Node):
    def __init__(self):
        super().__init__('lat_ab', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.state, self.rows, self.last = None, [], None                # rows: (sim arrival, latency, gap)
        self.create_subscription(String, 'gn_supervisor/state', lambda m: setattr(self, 'state', json.loads(m.data)['state']), LATCHED)
        self.create_subscription(Odometry, '/Odometry', self.on_odom, BEST_EFFORT)

    def sim(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, m):
        now, stamp = self.sim(), m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.rows.append((now, now - stamp, now - self.last if self.last else 0.))
        self.last = now

    def window(self, t0, t1):
        r = [x for x in self.rows if t0 <= x[0] <= t1]
        return summarize([x[1] for x in r], [x[2] for x in r][1:])

    def spin_for(self, s):
        t0 = self.sim()
        while rclpy.ok() and self.sim() - t0 < s:
            rclpy.spin_once(self, timeout_sec=.05)


def main():
    args = sys.argv[1:]
    zb = []
    if '--z-band' in args:
        i = args.index('--z-band'); zb = ['--z-band', args[i + 1], args[i + 2]]; del args[i:i + 3]
    out, inp = args[0], args[1]
    reps, idle_s = int(args[2]) if len(args) > 2 else 4, float(args[3]) if len(args) > 3 else 30.
    rclpy.init()
    n = Probe()
    t0 = n.sim()
    while rclpy.ok() and n.state != 'MONITOR' and n.sim() - t0 < 300:
        rclpy.spin_once(n, timeout_sec=.1)
    res = {'state': n.state}
    n.spin_for(15.)                                                      # settle
    a = n.sim(); n.spin_for(idle_s)
    res['idle'] = n.window(a, n.sim())
    for mode in ('search', 'nice'):
        wins, wall = [], []
        for k in range(reps):
            cmd = ['nice', '-n', '10'] if mode == 'nice' else []
            cmd += [sys.executable, '-B', '/greloc/tools/global_reloc_3d/search_v2.py', '--run', inp, '--out', f'{out}.{mode}{k}', '--binary', '/greloc/build/register_v2',
                    '--deadline-s', '60', '--threads', '2'] + zb
            w0, s0 = time.monotonic(), n.sim()
            p = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            while p.poll() is None:
                rclpy.spin_once(n, timeout_sec=.05)
            wall.append(round(time.monotonic() - w0, 2)); wins.append((s0, n.sim()))
            n.spin_for(8.)
        res[mode] = {'search_wall_s': wall, **summarize([x[1] for s, e in wins for x in n.rows if s <= x[0] <= e], [x[2] for s, e in wins for x in n.rows if s <= x[0] <= e])}
    json.dump(res, open(out, 'w'), indent=1)
    print(json.dumps(res, indent=1))
    rclpy.shutdown()


if __name__ == '__main__':
    main()
