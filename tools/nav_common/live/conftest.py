"""pytest tools/: test_live_paths.py needs rclpy (a laptop with ROS sourced, or the robot image); without it the file is left out of the collection."""
import importlib.util

collect_ignore = [] if importlib.util.find_spec("rclpy") else ["test_live_paths.py"]
