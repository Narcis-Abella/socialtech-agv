"""Independent witness subscriber: counts /livox/imu messages and header-stamp gaps until SIGTERM, writes JSON.
Separates publisher/transport loss (the witness sees it too) from loss inside the relocalizer. usage: topic_loss.py out.json"""
import json, signal, sys, time
import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Imu

rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
node = rclpy.create_node('topic_loss_witness')
stamps = []
node.create_subscription(Imu, '/livox/imu', lambda m: stamps.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9),
                         QoSProfile(depth=2000, reliability=ReliabilityPolicy.BEST_EFFORT))
stop = []
signal.signal(signal.SIGTERM, lambda *_: stop.append(1)); signal.signal(signal.SIGINT, lambda *_: stop.append(1))
while not stop:
    rclpy.spin_once(node, timeout_sec=.1)
t = np.asarray(stamps)
d = np.diff(t) if len(t) > 2 else np.array([.005])
period = float(np.median(d))
lost = int(np.sum(np.maximum(np.round(d / period) - 1, 0)))
json.dump({'topic': '/livox/imu', 'received': len(t), 'lost_estimate': lost, 'period_s': period,
           'loss_fraction': lost / max(1, len(t) + lost), 'max_gap_s': float(d.max())}, open(sys.argv[1], 'w'), indent=2)
