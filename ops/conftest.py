"""pytest ops/ : test_send_goal.py needs nav2_msgs (the robot image); without it the file is left out of the collection.
It is a plain script too (python3 src/bringup/socialtech_bringup/test/test_send_goal.py): there it prints a skip and exits 0."""
import importlib.util

collect_ignore = [] if importlib.util.find_spec("nav2_msgs") else ["test_send_goal.py"]
