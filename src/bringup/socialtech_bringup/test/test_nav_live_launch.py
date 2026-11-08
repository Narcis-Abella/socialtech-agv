"""nav_live.launch.py wiring that the supervisor relies on (needs the ROS 2 python packages, so inside the image): python3 src/bringup/socialtech_bringup/test/test_nav_live_launch.py"""
import importlib.util
import os

try:
    from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
    get_package_share_directory("fast_lio")      # the extrinsic is checked against its config
    import launch_ros  # noqa: F401
except (ModuleNotFoundError, PackageNotFoundError) as e:      # fast_lio, launch_ros: only in the robot image
    if __name__ != "__main__":
        raise      # under pytest, conftest.py leaves these files out when the image is missing; getting here is a real failure
    print(f"skip: needs the robot image ({e})")
    raise SystemExit(0)

from launch import LaunchContext
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile

here = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("nav_live_launch", os.path.join(here, "..", "launch", "nav_live.launch.py"))
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

# bond_heartbeat_period lives in the overrides file: gn_loc and map_server must read it
def reads_overrides(executable):
    ctx = LaunchContext()
    ctx.launch_configurations["overrides"] = "OVERRIDES"
    return any(isinstance(p, ParameterFile) and perform_substitutions(ctx, p.param_file) == "OVERRIDES" for p in node(executable)._Node__parameters)


for exe in ("gn_loc_node", "map_server"):
    assert reads_overrides(exe), f"{exe} does not get the overrides file"

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

# viz: live_paths.py is a rviz aid; PROD=1 (nav_live.sh) turns it off with viz:=false, the default keeps it
from launch.actions import ExecuteProcess


def live_paths():
    ctx = LaunchContext()
    ctx.launch_configurations["map"] = "m.yaml"
    for e in launch.generate_launch_description().entities:
        if type(e) is ExecuteProcess and any("live_paths" in perform_substitutions(ctx, c) for c in e.cmd):
            return e
    raise AssertionError("no live_paths process in nav_live.launch.py")


def runs(entity, viz):
    ctx = LaunchContext()
    ctx.launch_configurations["viz"] = viz
    return entity.condition is None or entity.condition.evaluate(ctx)


from launch.actions import DeclareLaunchArgument
viz_default = [e for e in launch.generate_launch_description().entities if isinstance(e, DeclareLaunchArgument) and e.name == "viz"]
assert viz_default and perform_substitutions(LaunchContext(), viz_default[0].default_value) == "true", "viz must default to true (DEV)"
assert runs(live_paths(), "true"), "live_paths must run with viz:=true"
assert not runs(live_paths(), "false"), "viz:=false must stop live_paths"
print("ok viz")

# /scan comes from the stock pointcloud_to_laserscan (the quantile-0 scan_median package was removed: it did the same, and the median it was written for was rejected)
launch_text = [e for e in launch.generate_launch_description().entities if isinstance(e, Node)]
assert any(e.node_executable == "pointcloud_to_laserscan_node" for e in launch_text), "no pointcloud_to_laserscan_node"
assert not any(e.node_package == "scan_median" for e in launch_text), "scan_median must be gone"
# /scan at the height of the lidar, the same band above the floor (scan.yaml): a scan on the floor sits on the costmaps' min_obstacle_height 0 and any error in z drops it
import yaml

ctx = LaunchContext()
ctx.launch_configurations["sensor_height"] = "0.5"
p2l = node("pointcloud_to_laserscan_node")
extra = [p for p in p2l._Node__parameters if isinstance(p, dict)][0]
values = {perform_substitutions(ctx, k): v for k, v in extra.items()}
values = {k: (v.evaluate(ctx) if hasattr(v, "evaluate") else v) for k, v in values.items()}
assert yaml.safe_load(perform_substitutions(ctx, values["target_frame"])) == "lidar_level", values   # launch_ros keeps a plain string as YAML
h = 0.5 + 0.04412
assert abs(values["min_height"] - (0.10 - h)) < 1e-9 and abs(values["max_height"] - (1.80 - h)) < 1e-9, values
print("ok scan height")
ctx.launch_configurations.update(map="MAP", sim_time="false")
assert any(type(e) is ExecuteProcess and any("local_frame_aligner" in perform_substitutions(ctx, c) for c in e.cmd) for e in launch.generate_launch_description().entities), "no local_frame_aligner: the local costmap has no frame"
print("ok aligner")


# level_corrector: off by default (planar_odom keeps the static level), on = planar_odom listens to the correction and the node starts with the same level
def planar_param(name, level_corrector):
    ctx = LaunchContext()
    ctx.launch_configurations.update({"level_corrector": level_corrector, "level_qx": "0", "level_qy": "0", "level_qz": "0", "level_qw": "1", "sensor_height": "0.5"})
    p = node("planar_odom")._Node__parameters[0]
    v = next(val for key, val in p.items() if "".join(k.text for k in key) == name)   # launch_ros keys the dict by tuples of TextSubstitution
    from launch_ros.parameter_descriptions import ParameterValue
    return perform_substitutions(ctx, v.value) if isinstance(v, ParameterValue) else v


assert planar_param("correction_topic", "false") == "", "planar_odom must keep the static level unless level_corrector:=true"
assert planar_param("correction_topic", "true") == "/level_corrector/correction"
lc_node = node("level_corrector_node")
assert lc_node.condition is not None, "level_corrector_node must be conditional on level_corrector:=true"
def cond(v):
    ctx = LaunchContext()
    ctx.launch_configurations["level_corrector"] = v
    return lc_node.condition.evaluate(ctx)


assert cond("true") and not cond("false")
print("ok level_corrector")

# a crashed corrector comes back (planar_odom drops a C that goes silent, then the corrector re-primes); same policy as FAST-LIO2
assert respawns(node("level_corrector_node")), "level_corrector_node is not respawned"
print("ok respawn corrector")

# the lidar offset in the body is copied in the launch and in sim_up.sh: it must be FAST-LIO2's extrinsic_T of the image (nav_live.sh edits the config but not that key)
import re
from ament_index_python.packages import get_package_share_directory
fl = yaml.safe_load(open(os.path.join(get_package_share_directory("fast_lio"), "config", "mid360.yaml")))
ext = list(fl["/**"]["ros__parameters"]["mapping"]["extrinsic_T"])
assert ext == launch.LIDAR_IN_BODY, f"launch LIDAR_IN_BODY {launch.LIDAR_IN_BODY} != FAST-LIO2 extrinsic_T {ext}"
sim_up = open(os.path.join(here, "..", "..", "..", "..", "tools", "nav_sim", "sim_up.sh")).read()
m = re.search(r"lidar_in_body:=\"\[([^\]]*)\]\"", sim_up)
assert m and [float(x) for x in m.group(1).split(",")] == ext, f"sim_up.sh lidar_in_body {m and m.group(1)} != extrinsic_T {ext}"
print("ok extrinsic")
