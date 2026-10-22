"""Test-only stand-in for a velocity source and a witness of the gate: publishes a constant non-zero Twist on the gate input for the
whole run and logs, with monotonic receipt times, every change on the gated output, the permit and the node state.
No base driver runs in the bench containers, so nothing can move.   gate_probe.py OUT.json [cmd_in] [cmd_out]"""
import json
import signal
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.clock import Clock, ClockType
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, String


def main():
    out = sys.argv[1]
    cmd_in = sys.argv[2] if len(sys.argv) > 2 else '/cmd_vel'
    cmd_out = sys.argv[3] if len(sys.argv) > 3 else '/cmd_vel_gated'
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('gate_probe')
    log = {'sent': 0, 'out_nonzero': 0, 'out_zero': 0, 'nonzero_intervals': [], 'permit_edges': [], 'states': [], 'gate_status_edges': [],
           'permit_true': 0, 'permit_false': 0, 'max_permit_gap_while_true_s': 0.}
    last = {'permit': None, 'permit_t': None, 'nz': False, 'gate': None}
    pub = node.create_publisher(Twist, cmd_in, 5)

    def send():
        m = Twist(); m.linear.x = .1; m.angular.z = .05
        pub.publish(m); log['sent'] += 1

    def on_out(m):
        t = time.monotonic()
        nz = bool(m.linear.x or m.linear.y or m.angular.z)
        log['out_nonzero' if nz else 'out_zero'] += 1
        if nz and not last['nz']:
            log['nonzero_intervals'].append([t, t])
        elif nz:
            log['nonzero_intervals'][-1][1] = t
        last['nz'] = nz

    def on_permit(m):
        t = time.monotonic()
        log['permit_true' if m.data else 'permit_false'] += 1
        if m.data and last['permit'] and last['permit_t'] is not None:
            log['max_permit_gap_while_true_s'] = max(log['max_permit_gap_while_true_s'], t - last['permit_t'])
        if m.data != last['permit']:
            log['permit_edges'].append([t, bool(m.data)])
        last['permit'], last['permit_t'] = m.data, t

    def on_state(m):
        d = json.loads(m.data)
        log['states'].append([time.monotonic(), d.get('state'), d.get('reason')])

    def on_gate(m):
        d = json.loads(m.data)
        if (d['open'], d['reason']) != last['gate']:
            last['gate'] = (d['open'], d['reason'])
            log['gate_status_edges'].append([d['mono'], d['open'], d['reason'], d.get('bypass_publishers')])
        log['gate_last'] = d

    node.create_subscription(Twist, cmd_out, on_out, 50)
    node.create_subscription(Bool, '/global_reloc/nav_permit', on_permit, 50)
    node.create_subscription(String, '/global_reloc/state', on_state, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(String, '/nav_gate/status', on_gate, 50)
    node.create_timer(.1, send, clock=Clock(clock_type=ClockType.STEADY_TIME))
    stop = []
    for s in (signal.SIGINT, signal.SIGTERM):
        signal.signal(s, lambda *_: stop.append(1))
    while not stop and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=.1)
    log['end_mono'] = time.monotonic()
    with open(out, 'w') as f:
        json.dump(log, f, indent=1)


if __name__ == '__main__':
    main()
