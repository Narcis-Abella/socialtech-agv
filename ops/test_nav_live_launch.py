"""nav_live.launch.py wiring that the supervisor relies on (needs the ROS 2 python packages, so inside the image): python3 ops/test_nav_live_launch.py"""
import importlib.util
import os

from launch import LaunchContext
from launch.utilities import perform_substitutions
from launch_ros.actions import Node

here = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("nav_live_launch", os.path.join(here, "nav_live.launch.py"))
launch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launch)


def node(executable):
    for e in launch.generate_launch_description().entities:
        if isinstance(e, Node) and e.node_executable == executable:
            return e
    raise AssertionError(f"no {executable} node in nav_live.launch.py")


def respawns(n):
    r = n._ExecuteLocal__respawn                      # a bool when given as one, a list of substitutions when given as a launch argument
    return r is True or (not isinstance(r, bool) and perform_substitutions(LaunchContext(), r).lower() == "true")


# the supervisor restarts FAST-LIO2 by killing it: the launch must bring it back (and it does so after a crash too)
assert respawns(node("fastlio_mapping")), "fastlio_mapping is not respawned"
# the others keep their behaviour: a dying Livox driver must stay visible, not be restarted silently under a moving robot
assert not respawns(node("livox_ros_driver2_node")), "the Livox driver must not respawn"
print("ok")
