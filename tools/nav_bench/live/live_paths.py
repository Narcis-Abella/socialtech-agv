"""Live view helper (runs on the board next to the bench): publishes the Voxel-SLAM reference as a latched nav_msgs/Path on /ref_path, the AMCL trail as /amcl_path and
AMCL's particles (nav2_msgs/ParticleCloud on /particle_cloud, which rviz2 cannot draw) as a geometry_msgs/PoseArray on /particle_poses.
usage: live_paths.py <alidarState.txt | -> <map.yaml> --ros-args -p use_sim_time:=true"""
import math, os, sys
import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.msg import ParticleCloud
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
from pose_eval import read_map_yaml, reference_map

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
        self.trail_pub = self.create_publisher(Path, "/amcl_path", 1)
        self.trail = Path()
        self.trail.header.frame_id = "map"
        if ref != "-":   # "-": bag over another bag's map (stage B), the reference would sit in the wrong place
            _, r = reference_map(ref, read_map_yaml(map_yaml)["T_map_world"])
            msg = Path()
            msg.header.frame_id, msg.header.stamp = "map", self.get_clock().now().to_msg()
            msg.poses = [pose(x, y, yaw, msg.header.stamp) for x, y, yaw in r[::5]]   # 10 Hz reference -> every 0.5 s
            self.ref_pub.publish(msg)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.on_pose, 10)
        self.poses_pub = self.create_publisher(PoseArray, "/particle_poses", qos_profile_sensor_data)
        self.create_subscription(ParticleCloud, "/particle_cloud", self.on_particles, qos_profile_sensor_data)

    def on_particles(self, m):
        a = PoseArray()
        a.header, a.poses = m.header, [p.pose for p in m.particles]
        self.poses_pub.publish(a)

    def on_pose(self, m):
        p = PoseStamped()
        p.header, p.pose = m.header, m.pose.pose
        self.trail.poses = (self.trail.poses + [p])[-3000:]
        self.trail.header.stamp = m.header.stamp
        self.trail_pub.publish(self.trail)


def main():
    rclpy.init(args=sys.argv)
    rclpy.spin(Paths(sys.argv[1], sys.argv[2]))


if __name__ == "__main__":
    main()
