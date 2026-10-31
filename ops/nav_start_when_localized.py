"""Start Nav2 once the robot is localized. Nav2's lifecycle manager gives up for good when the global costmap times out waiting for map -> base_footprint, and that
transform only exists after AMCL gets its initial pose (the operator's 2D Pose Estimate in rviz2), so Nav2 is launched with autostart:=false and started here."""
import sys

import rclpy
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

rclpy.init()
node = Node("nav_start_when_localized")
buf = Buffer()
TransformListener(buf, node)
client = node.create_client(ManageLifecycleNodes, "/lifecycle_manager_navigation/manage_nodes")
node.get_logger().info("waiting for the initial pose (2D Pose Estimate in rviz2) before starting Nav2")
while rclpy.ok() and not buf.can_transform("map", "base_footprint", rclpy.time.Time()):
    rclpy.spin_once(node, timeout_sec=0.5)
node.get_logger().info("localized: starting Nav2")
client.wait_for_service()
future = client.call_async(ManageLifecycleNodes.Request(command=ManageLifecycleNodes.Request.STARTUP))
rclpy.spin_until_future_complete(node, future)
ok = future.result() is not None and future.result().success
node.get_logger().info("Nav2 started" if ok else "Nav2 startup FAILED (see the lifecycle_manager_navigation log)")
sys.exit(0 if ok else 1)
