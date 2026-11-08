"""Synthetic inputs for gn_loc: a 5 m square room map, odom->base_footprint TF at 20 Hz, /scan at SCAN_HZ (default 10) seen from the room centre.
/initialpose is republished every second until gn_loc/scan_fit reports TRACKING (a single volatile publish can be lost if gn_loc configures late);
then 'SYNTH_TRACKING' is printed once, the harness's go signal for the stop. Runs until SIGINT."""
import math, os, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
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
        self.map_pub = self.create_publisher(OccupancyGrid, 'map', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
        self.scan_pub = self.create_publisher(LaserScan, 'scan', qos_profile_sensor_data)
        self.init_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self.create_subscription(String, 'gn_loc/scan_fit', self.on_fit, 10)
        self.br = TransformBroadcaster(self)
        self.map_pub.publish(m)
        self.tracking = False; self.inits = 0
        self.create_timer(1.0 / float(os.environ.get('SCAN_HZ', '10')), self.tick); self.create_timer(0.05, self.tf); self.create_timer(1.0, self.init)

    def on_fit(self, msg):
        if not self.tracking and msg.data.startswith('TRACKING'):
            self.tracking = True
            print(f'SYNTH_TRACKING after {self.inits} initialpose', flush=True)

    def tf(self):
        t = TransformStamped(); t.header.stamp = self.get_clock().now().to_msg(); t.header.frame_id = 'odom'; t.child_frame_id = 'base_footprint'; t.transform.rotation.w = 1.
        self.br.sendTransform(t)

    def init(self):
        if self.tracking:
            return
        p = PoseWithCovarianceStamped(); p.header.stamp = self.get_clock().now().to_msg(); p.header.frame_id = 'map'; p.pose.pose.orientation.w = 1.
        p.pose.covariance[0] = p.pose.covariance[7] = 0.01; p.pose.covariance[35] = 0.01
        self.init_pub.publish(p); self.inits += 1

    def tick(self):
        s = LaserScan(); s.header.stamp = self.get_clock().now().to_msg(); s.header.frame_id = 'base_footprint'
        s.angle_min, s.angle_max, s.angle_increment = -math.pi, math.pi, 2 * math.pi / 360
        s.range_min, s.range_max = 0.1, 12.
        half = 120 * 0.05 / 2 - 2 * 0.05
        s.ranges = [half / max(abs(math.cos(s.angle_min + i * s.angle_increment)), abs(math.sin(s.angle_min + i * s.angle_increment))) for i in range(360)]
        self.scan_pub.publish(s)

rclpy.init(); n = Synth()
try:
    rclpy.spin(n)
except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
    pass
