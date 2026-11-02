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

# HEAVY_DDS: off by default (nothing changes for the driver and FAST-LIO2), on = a per-node loopback URI with 64 kB fragments for exactly those two nodes
def env_of(executable):
    return node(executable).additional_env


os.environ.pop("HEAVY_DDS", None)
assert not launch.heavy_env(), "HEAVY_DDS must be off by default"
os.environ["HEAVY_DDS"] = "1"
heavy = launch.heavy_env()
assert "FragmentSize>64000B" in heavy["additional_env"]["CYCLONEDDS_URI"] and "SocketReceiveBufferSize" in heavy["additional_env"]["CYCLONEDDS_URI"]
for exe in ("fastlio_mapping", "livox_ros_driver2_node"):
    assert env_of(exe), f"{exe} does not get the heavy URI"
assert not env_of("planar_odom"), "only the two ends of the lidar topic get it"
os.environ["HEAVY_DDS"] = "0"
assert not launch.heavy_env()
print("ok heavy")
