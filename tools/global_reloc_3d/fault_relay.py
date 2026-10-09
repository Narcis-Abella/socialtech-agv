"""Fault injector between the bag player (topics remapped to <topic>_raw) and the relocalizer: forwards serialized messages untouched
except for the requested fault. Used only by robustness tests; the relocalizer never knows.
  fault_relay.py --topic /livox/lidar --type livox_ros_driver2/msg/CustomMsg [--drop P] [--shift-s S] [--shift-after-s T] [--seed N]
Both topics are relayed by one process: repeat --topic/--type pairs with the same fault options per pair via --spec topic,type,drop,shift_s"""
import argparse
import random
import struct
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rosidl_runtime_py.utilities import get_message

QOS = QoSProfile(depth=2000, reliability=ReliabilityPolicy.BEST_EFFORT)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--spec', action='append', required=True, help='topic,type,drop_prob,stamp_shift_s')
    p.add_argument('--seed', type=int, default=1)
    a = p.parse_args()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('fault_relay')
    rng = random.Random(a.seed)
    stats = {}
    for spec in a.spec:
        topic, typ, drop, shift = spec.split(',')
        drop, shift = float(drop), float(shift)
        msg_type = get_message(typ)
        pub = node.create_publisher(msg_type, topic, QOS)
        stats[topic] = [0, 0]

        def cb(raw, pub=pub, topic=topic, drop=drop, shift=shift):
            stats[topic][0] += 1
            if drop and rng.random() < drop:
                stats[topic][1] += 1
                return
            if shift:   # patch Header.stamp.sec inside the CDR bytes (offset 4 after the encapsulation header), no deserialization
                b = bytearray(raw)
                fmt = '<i' if b[1] == 1 else '>i'
                sec = struct.unpack_from(fmt, b, 4)[0]
                struct.pack_into(fmt, b, 4, sec + int(shift))
                raw = bytes(b)
            pub.publish(raw)
        node.create_subscription(msg_type, topic + '_raw', cb, QOS, raw=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    print('relay stats (received, dropped):', stats, flush=True)


if __name__ == '__main__':
    main()
