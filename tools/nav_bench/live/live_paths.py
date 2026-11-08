"""Live view helper (runs on the board next to the bench): publishes the Voxel-SLAM reference as a latched nav_msgs/Path on /ref_path and gn_loc's trail as /gn_path.
usage: live_paths.py <alidarState.txt | -> <map.yaml> --ros-args -p use_sim_time:=true"""
import math, os, sys
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
from map_frame import read_map_yaml, reference_map

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)


def pose(x, y, yaw, stamp):
    p = PoseStamped()
    p.header.frame_id, p.header.stamp = "map", stamp
    p.pose.position.x, p.pose.position.y = float(x), float(y)
    p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return p


class Paths(Node):
    def __init__(self, ref, map_yaml):
        super().__init__("live_paths")
        self.ref_pub = self.create_publisher(Path, "/ref_path", LATCHED)
        self.trail_pub = self.create_publisher(Path, "/gn_path", 1)
        self.trail = Path()
        self.trail.header.frame_id = "map"
        if ref != "-":   # "-": bag over another bag's map (stage B), the reference would sit in the wrong place
            _, r = reference_map(ref, read_map_yaml(map_yaml)["T_map_world"])
            msg = Path()
            msg.header.frame_id, msg.header.stamp = "map", self.get_clock().now().to_msg()
            msg.poses = [pose(x, y, yaw, msg.header.stamp) for x, y, yaw in r[::5]]   # 10 Hz reference -> every 0.5 s
            self.ref_pub.publish(msg)
        self.create_subscription(PoseWithCovarianceStamped, "/gn_loc/pose", self.on_pose, 10)

    def on_pose(self, m):
        p = PoseStamped()
        p.header, p.pose = m.header, m.pose.pose
        self.trail.poses = (self.trail.poses + [p])[-3000:]
        self.trail.header.stamp = m.header.stamp
        self.trail_pub.publish(self.trail)


def spin(node):
    try:
        rclpy.spin(node)
    except Exception:
        if rclpy.ok():
            raise
        # the launch stopped: a pose that arrived with the context already gone fails to publish; that is the end of the node, not an error (it used to exit with code 1)


def main():
    rclpy.init(args=sys.argv)
    spin(Paths(sys.argv[1], sys.argv[2]))


if __name__ == "__main__":
    main()
