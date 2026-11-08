"""odom_aligned: a child of odom, rotated so that its axes are those of map, where the local costmap lives. A rolling window keeps the axes of its frame, odom's come from where FAST-LIO2 started, and the map is
aligned with the walls: in this frame the walls stay straight rows of cells. It depends on the localizer only for this one angle, so the costmap keeps odom's properties: marks do not move with the
localizer, and a frozen localizer stops the controller (RPP's plan transform goes through map -> odom; with the costmap in map it does not, and the robot drove 5.03 m on a frozen pose).
Aligns at the first map -> odom. Then, parked (still_s without moving), when off by more than parked_yaw_deg: that corrects the unconfirmed first pose and the drift (3.5 deg in 9 min) at every stop. Driving, only on a
jump past jump_yaw_deg (FAST-LIO2 restarted), because the frame turns about odom's origin: 10 m away, 5 deg move the pose 0.87 m until the clear. Either must last persist_s: one wild sample is no reason. A rotation moves the
marks with it, so the local costmap is cleared after it (once the new TF has arrived). Stopped: the last alignment stays."""
import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformException, TransformListener


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Aligner(Node):
    def __init__(self):
        super().__init__("local_frame_aligner")
        P = self.declare_parameter
        self.parked_yaw = math.radians(P("parked_yaw_deg", 1.0).value)
        self.jump_yaw = math.radians(P("jump_yaw_deg", 15.0).value)
        self.still_s, self.persist_s = P("still_s", 2.0).value, P("persist_s", 0.5).value
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        # not tf2_ros.StaticTransformBroadcaster: its Python one (Jazzy) silently keeps the first transform of a child frame, and this one changes
        self.static = self.create_publisher(TFMessage, "/tf_static", QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.clear = self.create_client(ClearEntireCostmap, "/local_costmap/clear_entirely_local_costmap")
        self.yaw = None           # map -> odom yaw of the alignment in force
        self.anchor = None        # (x, y, yaw, t) where the robot last moved
        self.t_due = None         # since when an alignment is due
        self.t_clear = None       # since when a clear is pending
        self.clear_timer = self.create_timer(P("clear_delay_s", 0.3).value, self.clear_local)
        self.clear_timer.cancel()
        self.create_timer(0.1, self.tick)

    def parked(self, t):
        try:
            b = self.buf.lookup_transform("odom", "base_footprint", rclpy.time.Time()).transform
        except TransformException:
            return False
        x, y, yaw = b.translation.x, b.translation.y, yaw_of(b.rotation)
        if self.anchor is None or math.hypot(x - self.anchor[0], y - self.anchor[1]) > 0.01 or abs(wrap(yaw - self.anchor[2])) > math.radians(0.5):
            self.anchor = (x, y, yaw, t)
        return t - self.anchor[3] >= self.still_s

    def tick(self):
        try:
            yaw = yaw_of(self.buf.lookup_transform("map", "odom", rclpy.time.Time()).transform.rotation)
        except TransformException:
            return                # not localized yet
        t = self.get_clock().now().nanoseconds * 1e-9
        parked = self.parked(t)
        if self.yaw is None:
            self.get_logger().info(f"aligning: map -> odom yaw {math.degrees(yaw):.2f} deg")
            return self.align(yaw, t)
        off = abs(wrap(yaw - self.yaw))
        if off <= (self.parked_yaw if parked else self.jump_yaw):
            self.t_due = None
            return
        self.t_due = t if self.t_due is None else self.t_due
        if t - self.t_due >= self.persist_s:
            self.get_logger().info(f"aligning: map -> odom yaw {math.degrees(yaw):.2f} deg, {math.degrees(off):.2f} off the last alignment" + (", parked" if parked else ", driving"))
            self.align(yaw, t)

    def align(self, yaw, t):
        self.yaw, self.t_due, self.t_clear = yaw, None, t
        m = TransformStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id, m.child_frame_id = "odom", "odom_aligned"
        m.transform.rotation.z, m.transform.rotation.w = math.sin(-yaw / 2), math.cos(-yaw / 2)
        self.static.publish(TFMessage(transforms=[m]))
        self.clear_timer.reset()

    def clear_local(self):
        if self.clear.service_is_ready():
            self.clear.call_async(ClearEntireCostmap.Request())
        elif self.get_clock().now().nanoseconds * 1e-9 - self.t_clear < 5.0:
            return                # discovery after a respawn: the marks of the old alignment must not stay
        else:
            self.get_logger().info("local costmap not cleared: its service is not there (normal before Nav2 starts)")
        self.clear_timer.cancel()


def main():
    rclpy.init()
    try:
        rclpy.spin(Aligner())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
