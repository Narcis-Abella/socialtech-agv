"""Driver-cell side of the navigation gate: the only publisher of /cmd_vel that tracer_base can see.

Runs in the same private network namespace as tracer_base. Commands arrive from nav_gate.py as datagrams on a unix socket whose
directory is mounted only into the gate and into this cell. No datagram for OUTLET_TTL_S -> zero Twist at 10 Hz for as long as
the silence lasts: a dead gate, a dead velocity source or a closed gate with nothing to block all end in commanded zeros, not in a
retained last command. Datagrams that are malformed, out of sequence or older than the TTL are dropped and counted.
  gate_outlet.py SOCKET_PATH [STATS.json]"""
import json
import os
import select
import signal
import socket
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.signals import SignalHandlerOptions

sys.path.insert(0, str(Path(__file__).resolve().parent))
from nav_gate import MAGIC, WIRE

OUTLET_TTL_S = .3        # three missed commands at 10 Hz
ZERO_PERIOD_S = .1


def main():
    path, stats_path = sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else None
    if os.path.exists(path):
        os.unlink(path)
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    sock.bind(path); os.chmod(path, 0o600)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('gate_outlet')
    pub = node.create_publisher(Twist, '/cmd_vel', 5)
    n = {'forwarded': 0, 'nonzero_forwarded': 0, 'zero_fill': 0, 'dropped_malformed': 0, 'dropped_sequence': 0, 'dropped_stale': 0}
    stop = []
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.append(1))
    last_seq, last_rx, last_zero = 0, -1e9, 0.
    while not stop:
        if select.select([sock], [], [], .02)[0]:
            data = sock.recv(256)
            now = time.monotonic()
            if len(data) != WIRE.size or WIRE.unpack(data)[4] != MAGIC:
                n['dropped_malformed'] += 1
                continue
            seq, sent, vx, wz, _ = WIRE.unpack(data)
            if seq <= last_seq and seq != 1:          # seq 1 = a restarted gate
                n['dropped_sequence'] += 1
                continue
            last_seq = seq
            if not (0. <= now - sent <= OUTLET_TTL_S) or vx != vx or wz != wz:      # same host, same monotonic clock; NaN never forwarded
                n['dropped_stale'] += 1
                continue
            m = Twist(); m.linear.x, m.angular.z = vx, wz
            pub.publish(m)
            last_rx = now
            n['forwarded'] += 1; n['nonzero_forwarded'] += bool(vx or wz)
        now = time.monotonic()
        if now - last_rx > OUTLET_TTL_S and now - last_zero >= ZERO_PERIOD_S:
            pub.publish(Twist()); last_zero = now
            n['zero_fill'] += 1
    if stats_path:
        json.dump(n, open(stats_path, 'w'))


if __name__ == '__main__':
    main()
