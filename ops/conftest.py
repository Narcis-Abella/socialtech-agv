"""pytest ops/ : the tests that need the robot image (rclpy, nav2_msgs, fast_lio) are left out of the collection where it is missing.
They are plain scripts too (python3 ops/test_x.py): there they print a skip and exit 0."""
import importlib.util

collect_ignore = []
if importlib.util.find_spec("nav2_msgs") is None:
    collect_ignore += ["test_send_goal.py", "test_local_frame_aligner.py"]
if importlib.util.find_spec("launch") is None:
    collect_ignore += ["test_supervisor_launch.py"]
try:
    from ament_index_python.packages import get_package_share_directory
    get_package_share_directory("fast_lio")
except Exception:
    collect_ignore += ["test_nav_live_launch.py"]
