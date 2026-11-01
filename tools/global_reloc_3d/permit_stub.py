"""Test-only stand-in for the relocalizer's permit heartbeat (same node name, topic and rate), driven by a script of phases.
  permit_stub.py LOG.json value:seconds [value:seconds ...]      value: true | false | silent (alive, publishes nothing)
The process exits after the last phase without a final `false`: that is the "publisher died" case."""
import json
import sys
import time

import rclpy
from std_msgs.msg import Bool


def main():
    rclpy.init()
    node = rclpy.create_node('global_reloc')
    pub = node.create_publisher(Bool, '/global_reloc/nav_permit', 10)
    time.sleep(1.)
    log = []
    for phase in sys.argv[2:]:
        value, seconds = phase.split(':')
        t0 = time.time(); log.append({'value': value, 'start_epoch': t0, 'seconds': float(seconds)})
        while time.time() - t0 < float(seconds):
            if value != 'silent':
                m = Bool(); m.data = value == 'true'; pub.publish(m)
            time.sleep(.1)
    log.append({'value': 'exit', 'start_epoch': time.time(), 'seconds': 0.})
    json.dump(log, open(sys.argv[1], 'w'), indent=1)


if __name__ == '__main__':
    main()
