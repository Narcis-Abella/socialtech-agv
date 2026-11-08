"""pytest tools/: test_trials.py imports rclpy and nav2_msgs through trials.py (the robot image); on a laptop without either the file is left out of the collection."""
import importlib.util

collect_ignore = [] if importlib.util.find_spec("rclpy") and importlib.util.find_spec("nav2_msgs") else ["test_trials.py"]
