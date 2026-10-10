"""Fault injector between the bag player (topics remapped to <topic>_raw) and the relocalizer: forwards serialized messages untouched
except for the requested fault. Used only by robustness tests; the relocalizer never knows.
  fault_relay.py --spec topic,type,drop_prob,stamp_shift_s[,key=value...] [--spec ...] [--seed N]
Optional keys, per topic. Times are seconds of wall time since the first message of that topic:
  after=T until=T   the faults below and drop/shift act only inside [after, until)
  dup=P             send the message twice (duplicated stamp)
  swap=P            hold the message and send it after the next one (reversed stamp order)
  gyrobias=B        sensor_msgs/Imu only: add B rad/s to every gyro axis (drives FAST-LIO2 to diverge)
  accscale=K        sensor_msgs/Imu only: multiply the acceleration by K"""
import argparse
import random
import struct
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message, serialize_message
from rclpy.signals import SignalHandlerOptions
from rosidl_runtime_py.utilities import get_message

QOS = QoSProfile(depth=2000, reliability=ReliabilityPolicy.BEST_EFFORT)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--spec', action='append', required=True, help='topic,type,drop_prob,stamp_shift_s[,key=value...]')
    p.add_argument('--seed', type=int, default=1)
    a = p.parse_args()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('fault_relay')
    rng = random.Random(a.seed)
    stats = {}
    for spec in a.spec:
        topic, typ, drop, shift, *rest = spec.split(',')
        drop, shift = float(drop), float(shift)
        o = {'after': 0., 'until': float('inf'), 'dup': 0., 'swap': 0., 'gyrobias': 0., 'accscale': 1.}
        o.update({k: float(v) for k, v in (r.split('=') for r in rest)})
        msg_type = get_message(typ)
        pub = node.create_publisher(msg_type, topic, QOS)
        st = stats[topic] = {'received': 0, 'dropped': 0, 'shifted': 0, 'duplicated': 0, 'swapped': 0, 'altered': 0, 't0': None, 'held': None}

        def cb(raw, pub=pub, st=st, drop=drop, shift=shift, o=o, msg_type=msg_type):
            now = time.monotonic()
            st['t0'] = st['t0'] or now
            st['received'] += 1
            if not o['after'] <= now - st['t0'] < o['until']:
                return pub.publish(raw)
            if drop and rng.random() < drop:
                st['dropped'] += 1
                return
            if shift:   # patch Header.stamp.sec inside the CDR bytes (offset 4 after the encapsulation header), no deserialization
                b = bytearray(raw)
                fmt = '<i' if b[1] == 1 else '>i'
                sec = struct.unpack_from(fmt, b, 4)[0]
                struct.pack_into(fmt, b, 4, sec + int(shift))
                raw = bytes(b); st['shifted'] += 1
            if o['gyrobias'] or o['accscale'] != 1.:
                m = deserialize_message(raw, msg_type)
                for k in 'xyz':
                    setattr(m.angular_velocity, k, getattr(m.angular_velocity, k) + o['gyrobias'])
                    setattr(m.linear_acceleration, k, getattr(m.linear_acceleration, k) * o['accscale'])
                raw = serialize_message(m); st['altered'] += 1
            if st['held'] is not None:       # the held (older) message goes out after this newer one
                pub.publish(raw); pub.publish(st['held']); st['held'] = None
                return
            if o['swap'] and rng.random() < o['swap']:
                st['held'] = raw; st['swapped'] += 1
                return
            pub.publish(raw)
            if o['dup'] and rng.random() < o['dup']:
                pub.publish(raw); st['duplicated'] += 1
        node.create_subscription(msg_type, topic + '_raw', cb, QOS, raw=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    for s in stats.values():
        s.pop('held')
    print('relay stats:', stats, flush=True)


if __name__ == '__main__':
    main()
