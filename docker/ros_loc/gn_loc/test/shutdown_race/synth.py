"""Synthetic inputs for gn_loc: a 5 m square room map, odom->base_footprint TF, /scan at 10 Hz seen from the room centre, /initialpose once. Runs until SIGINT."""
import math, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
from tf2_ros import TransformBroadcaster

class Synth(Node):
    def __init__(self):
        super().__init__('synth')
        n, res = 120, 0.05
        m = OccupancyGrid(); m.header.frame_id = 'map'; m.info.resolution = res; m.info.width = m.info.height = n
        m.info.origin.position.x = m.info.origin.position.y = -n * res / 2; m.info.origin.orientation.w = 1.
        d = [0] * (n * n)
        for i in range(n):
            for k in (0, 1, n - 2, n - 1):
                d[i * n + k] = 100; d[k * n + i] = 100
        m.data = d
        self.map = m
        self.map_pub = self.create_publisher(OccupancyGrid, 'map', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
        self.scan_pub = self.create_publisher(LaserScan, 'scan', qos_profile_sensor_data)
        self.init_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self.br = TransformBroadcaster(self)
        self.map_pub.publish(m)
        self.t0 = self.get_clock().now(); self.sent_init = False
        self.create_timer(0.1, self.tick); self.create_timer(0.05, self.tf)

    def tf(self):
        t = TransformStamped(); t.header.stamp = self.get_clock().now().to_msg(); t.header.frame_id = 'odom'; t.child_frame_id = 'base_footprint'; t.transform.rotation.w = 1.
        self.br.sendTransform(t)

    def tick(self):
        now = self.get_clock().now()
        s = LaserScan(); s.header.stamp = now.to_msg(); s.header.frame_id = 'base_footprint'
        s.angle_min, s.angle_max, s.angle_increment = -math.pi, math.pi, 2 * math.pi / 360
        s.range_min, s.range_max = 0.1, 12.
        half = 120 * 0.05 / 2 - 2 * 0.05
        s.ranges = [half / max(abs(math.cos(s.angle_min + i * s.angle_increment)), abs(math.sin(s.angle_min + i * s.angle_increment))) for i in range(360)]
        self.scan_pub.publish(s)
        if not self.sent_init and (now - self.t0).nanoseconds > 2e9:
            p = PoseWithCovarianceStamped(); p.header.stamp = now.to_msg(); p.header.frame_id = 'map'; p.pose.pose.orientation.w = 1.
            p.pose.covariance[0] = p.pose.covariance[7] = 0.01; p.pose.covariance[35] = 0.01
            self.init_pub.publish(p); self.sent_init = True

rclpy.init(); n = Synth()
try:
    rclpy.spin(n)
except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
    pass
