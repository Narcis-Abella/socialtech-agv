"""Planar odom -> base_footprint TF from FAST-LIO2's /Odometry (camera_init -> body), in the floor-levelled odom frame.
odom = camera_init rotated by `level` (static TF from the launch file; pose_eval.py level): z is the floor normal and the axes follow the walls, as in
the PGM. base_footprint sits under the body: same x, y and yaw (in odom), z = body z - sensor_height (the floor is z = 0 there), no roll/pitch.
A cloud transformed into base_footprint is therefore levelled and its z means 'height above the floor' (the frame ply_to_map cuts in).
usage: planar_odom.py --ros-args -p sensor_height:=0.575 -p level:="[qx,qy,qz,qw]" [-p use_sim_time:=true]"""
import numpy as np

import pose_eval as pe


def footprint(p, q, level, sensor_height):
    """(x, y, z, qz, qw) of base_footprint in odom. p, q: body position and quaternion (x, y, z, w) in camera_init; level: quaternion of odom<-camera_init."""
    R_l = pe.quat_to_mat(level)
    po, Ro = R_l @ np.asarray(p, float), R_l @ pe.quat_to_mat(q)
    yaw = np.arctan2(Ro[1, 0], Ro[0, 0])
    return po[0], po[1], po[2] - sensor_height, np.sin(yaw / 2), np.cos(yaw / 2)


def main():
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from tf2_ros import TransformBroadcaster

    class PlanarOdom(Node):
        def __init__(self):
            super().__init__("planar_odom")
            self.h = self.declare_parameter("sensor_height", 0.575).value
            self.level = list(self.declare_parameter("level", [0.0, 0.0, 0.0, 1.0]).value)
            self.br = TransformBroadcaster(self)
            self.create_subscription(Odometry, "/Odometry", self.cb, 100)

        def cb(self, m):
            p, q = m.pose.pose.position, m.pose.pose.orientation
            t = TransformStamped()
            t.header.stamp, t.header.frame_id, t.child_frame_id = m.header.stamp, "odom", "base_footprint"
            tr, ro = t.transform.translation, t.transform.rotation
            tr.x, tr.y, tr.z, ro.z, ro.w = footprint((p.x, p.y, p.z), (q.x, q.y, q.z, q.w), self.level, self.h)
            self.br.sendTransform(t)

    rclpy.init()
    rclpy.spin(PlanarOdom())


if __name__ == "__main__":
    main()
