"""Navigation gate: the only path from a velocity source to the base driver. Fail-closed consumer of /global_reloc/nav_permit.

Deployment that closes the bypass (validated on a virtual CAN bus, gate_cell_test.sh; note 21):
  cmd_in (/cmd_vel) -> [gate] -> unix datagram socket `outlet_socket` -> gate_outlet.py -> /cmd_vel -> tracer_base -> CAN
The driver and the outlet live in their own network namespace ("driver cell"): no ROS participant outside it can discover the
driver, so no topic of the main graph reaches the base. The socket directory is mounted only into the gate and the cell.
cmd_out (/cmd_vel_gated) is then only a monitor of what the gate let through.

Without `outlet_socket` (first F2 arrangement, kept for the replay harness where no driver runs): tracer_base is remapped to
cmd_out; anything else publishing on cmd_out bypasses the gate and is only reported (`bypass_publishers`).
tracer_base forwards every Twist to the CAN bus with no check and no timeout of its own (tracer_messenger.hpp, TwistCmdCallback).
Open only while rc.PermitGate says so (consecutive fresh `true` heartbeats from exactly one publisher, the relocalizer node;
`publishers_-N` in the status means N publishers of which at least one is foreign). Closed: every incoming command
is replaced by a zero Twist, and a 1 s burst of zeros is sent at the closing edge. Otherwise the gate is silent, so a base that is
pushed by hand is not actively held. If the gate itself dies nothing is published: the stop then depends on the command timeout of
the base firmware, which this repo has not verified.
"""
import json
import socket
import struct
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, String

sys.path.insert(0, str(Path(__file__).resolve().parent))
import reloc_core as rc

STOP_BURST_S = 1.
WIRE = struct.Struct('<QdddI')      # seq, sender monotonic time, linear x, angular z, magic. Shared with gate_outlet.py
MAGIC = 0x47415445


class NavGate(Node):
    def __init__(self):
        super().__init__('nav_gate')
        self.permit_topic = self.declare_parameter('permit_topic', '/global_reloc/nav_permit').value
        self.permit_node = self.declare_parameter('permit_node', 'global_reloc').value
        cmd_in = self.declare_parameter('cmd_in', '/cmd_vel').value
        self.cmd_out = self.declare_parameter('cmd_out', '/cmd_vel_gated').value
        self.gate = rc.PermitGate()
        self.open = False
        self.reason = 'no_permit'
        self.burst_until = 0.
        self.n = {'forwarded': 0, 'blocked': 0, 'zeros_sent': 0, 'opened': 0, 'closed': 0}
        self.pub = self.create_publisher(Twist, self.cmd_out, 5)
        self.outlet = self.declare_parameter('outlet_socket', '').value
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) if self.outlet else None
        if self.sock:
            self.sock.setblocking(False)
        self.seq = 0
        self.n.update(outlet_sent=0, outlet_errors=0)
        self.status = self.create_publisher(String, '/nav_gate/status', 10)
        self.create_subscription(Bool, self.permit_topic, lambda m: self.gate.on_permit(bool(m.data), time.monotonic()), 10)
        self.create_subscription(Twist, cmd_in, self.on_cmd, 5)
        steady = Clock(clock_type=ClockType.STEADY_TIME)      # never /clock: the TTL is wall time
        self.create_timer(.05, self.tick, clock=steady)
        self.create_timer(.5, self.report, clock=steady)

    def evaluate(self):
        now = time.monotonic()
        # Exactly one permit publisher, and it must be the relocalizer node. A node name can be forged on purpose (only SROS2 would
        # prevent that); this stops the accidental cases: a stray `ros2 topic pub`, a second instance, a leftover of an old run.
        pubs = self.get_publishers_info_by_topic(self.permit_topic)
        n = len(pubs) if all(i.node_name == self.permit_node for i in pubs) else -len(pubs)
        is_open, reason = self.gate.state(now, n)
        if is_open != self.open or reason != self.reason:
            if self.open and not is_open:
                self.burst_until = now + STOP_BURST_S
                self.n['closed'] += 1
            self.n['opened'] += is_open and not self.open
            self.open, self.reason = is_open, reason
            self.get_logger().info(f'gate {"OPEN" if is_open else "CLOSED"} ({reason}) mono={now:.3f}')
            self.report()
        return is_open

    def on_cmd(self, m):
        if self.evaluate():
            self.n['forwarded'] += 1
            return self.send(m)
        self.n['blocked'] += 1
        self.zero()

    def zero(self):
        self.n['zeros_sent'] += 1
        self.send(Twist())

    def send(self, m):
        self.pub.publish(m)
        if self.sock:                      # a command that cannot be delivered is dropped: the outlet zero-fills on silence
            self.seq += 1
            try:
                self.sock.sendto(WIRE.pack(self.seq, time.monotonic(), m.linear.x, m.angular.z, MAGIC), self.outlet)
                self.n['outlet_sent'] += 1
            except OSError:
                self.n['outlet_errors'] += 1

    def tick(self):
        if not self.evaluate() and time.monotonic() < self.burst_until:
            self.zero()

    def report(self):
        m = String()
        m.data = json.dumps({'open': self.open, 'reason': self.reason, 'mono': time.monotonic(),
                             'bypass_publishers': self.count_publishers(self.cmd_out) - 1, 'outlet': bool(self.sock), **self.n})
        self.status.publish(m)


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = NavGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
