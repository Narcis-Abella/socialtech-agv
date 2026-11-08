"""pytest src/bringup/socialtech_bringup/test: the modules under test are in ../scripts (not a package: they are run by path). The tests that need the robot image
(rclpy, nav2_msgs, fast_lio) are left out of the collection where it is missing. Each is a plain script too (python3 test_x.py): there it prints a skip and exits 0."""
import importlib.util
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))

collect_ignore = []
if importlib.util.find_spec("rclpy") is None:
    collect_ignore += ["test_live_paths.py", "test_planar_odom_node.py"]
if importlib.util.find_spec("nav2_msgs") is None:
    collect_ignore += ["test_local_frame_aligner.py", "test_send_goal.py"]
if importlib.util.find_spec("launch") is None:
    collect_ignore += ["test_supervisor_launch.py"]
try:
    from ament_index_python.packages import get_package_share_directory
    get_package_share_directory("fast_lio")
except Exception:
    collect_ignore += ["test_nav_live_launch.py"]
