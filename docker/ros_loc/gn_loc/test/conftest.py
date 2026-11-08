"""pytest: test_gn_loc_node.py runs the built gn_loc_node and needs ROS 2 (the robot image); without rclpy it is left out of the collection."""
import importlib.util

collect_ignore = [] if importlib.util.find_spec("rclpy") else ["test_gn_loc_node.py"]
