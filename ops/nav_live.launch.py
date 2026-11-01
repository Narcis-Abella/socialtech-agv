"""Live localization on the robot: Livox driver + FAST-LIO2 + levelled planar odom + /scan + map_server + AMCL, on wall time. Same nodes as the bench
(tools/nav_bench/launch/bench_a.launch.py, which runs on a bag's sim time) plus the driver; AMCL gets no initial pose (click 2D Pose Estimate in rviz2).
ros2 launch nav_live.launch.py livox_config:=C.json fastlio_params:=F map:=M.yaml sensor_height:=H level_qx:=.. level_qy:=.. level_qz:=.. level_qw:=.. overrides:=O.yaml"""
import os
from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

NAV_BENCH = os.path.join(os.path.dirname(os.path.dirname(os.path.realpath(__file__))), "tools/nav_bench")


SCAN = os.path.join(NAV_BENCH, "config/scan.yaml")
CLOUD = [("cloud_in", "/cloud_registered_body")]


def scan_nodes():
    """Default: scan_median (overlay), quantile 0 = the closest point per bin = the same /scan as pointcloud_to_laserscan, plus the median of each bin on /scan_med (recorded, used by nothing).
    SCAN_CONVERTER=p2l: pointcloud_to_laserscan as before."""
    if os.environ.get("SCAN_CONVERTER") == "p2l":
        return [Node(package="pointcloud_to_laserscan", executable="pointcloud_to_laserscan_node", parameters=[SCAN], remappings=CLOUD + [("scan", "/scan")])]
    return [Node(package="scan_median", executable="scan_median_node", name=name, parameters=[SCAN, {"quantile": q}], remappings=CLOUD + [("scan", topic)])
            for name, q, topic in (("scan_closest", 0.0, "/scan"),)]


def localizer_nodes(overrides):
    """Default: gn_loc (Gauss-Newton scan-to-map, built in the image). LOCALIZER=amcl: nav2_amcl in its place. Same /initialpose in, map->odom and /amcl_pose out."""
    gn = os.environ.get("LOCALIZER") != "amcl"
    return [Node(package="gn_loc", executable="gn_loc_node", name="gn_loc", output="screen") if gn else
            Node(package="nav2_amcl", executable="amcl", parameters=[os.path.join(NAV_BENCH, "config/amcl.yaml"), overrides]),
            Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_localization",
                 parameters=[{"autostart": True, "node_names": ["map_server", "gn_loc" if gn else "amcl"]}])]


def supervisor_nodes():
    """SUPERVISOR=1 (ops/nav_live.sh): gn_supervisor_node.py staged in /greloc. Respawned: a restarted supervisor comes up in CONFIRMING and holds Nav2 until
    gn_loc confirms its pose. It reads the robot's own FAST-LIO2 (/Odometry, /cloud_registered_body, /livox/imu): no second FAST-LIO (PR #14's reloc_node.py is not used)."""
    if os.environ.get("SUPERVISOR", "").lower() in ("", "0", "false", "no", "off"):    # the rule of ops/flags.sh
        return []
    prior = os.environ.get("SUP_PRIOR")
    if not prior:
        raise RuntimeError("SUPERVISOR is set but SUP_PRIOR (the 3D prior of the map) is not: start it with ops/nav_live.sh, which checks the staged files and sets it")
    return [ExecuteProcess(name="gn_supervisor", output="screen", respawn=True, respawn_delay=2.0, cmd=[
        "python3", "-B", "/greloc/tools/global_reloc_3d/gn_supervisor_node.py", "--ros-args",
        "-p", f"map_xyz:={prior}", "-p", "register_binary:=/greloc/build/register_v2", "-p", "lib_dir:=/greloc/build",
        "-p", "wheel_topic:=/odom", "-p", "work_dir:=/runs/gn_supervisor"])]


def generate_launch_description():
    args = [DeclareLaunchArgument(n) for n in ("livox_config", "fastlio_params", "map", "overrides", "sensor_height", "level_qx", "level_qy", "level_qz", "level_qw")]
    return LaunchDescription(args + [
        Node(package="livox_ros_driver2", executable="livox_ros_driver2_node", name="livox_lidar_publisher", parameters=[{
            "xfer_format": 1, "multi_topic": 0, "data_src": 0, "publish_freq": 10.0, "output_data_type": 0, "frame_id": "livox_frame",
            "lvx_file_path": "/tmp/none.lvx", "user_config_path": LC("livox_config"), "cmdline_input_bd_code": "livox0000000001"}]),
        # respawn: gn_supervisor restarts it by killing it (a fresh filter takes gravity from a still IMU and starts at z = 0, which a long blind period does not leave)
        Node(package="fast_lio", executable="fastlio_mapping", parameters=[LC("fastlio_params")], respawn=True, respawn_delay=1.0),
        Node(package="planar_odom", executable="planar_odom", parameters=[{
            "sensor_height": ParameterValue(LC("sensor_height"), value_type=float),
            "level": ParameterValue(["[", LC("level_qx"), ",", LC("level_qy"), ",", LC("level_qz"), ",", LC("level_qw"), "]"], value_type=List[float])}]),
        *scan_nodes(),
        Node(package="nav2_map_server", executable="map_server", parameters=[{"yaml_filename": LC("map")}]),
        *localizer_nodes(LC("overrides")),
        # AMCL trail (/amcl_path) and its particles as a PoseArray (/particle_poses), which rviz2 can draw; "-" = no reference path
        ExecuteProcess(cmd=["python3", os.path.join(NAV_BENCH, "live/live_paths.py"), "-", LC("map"), "--ros-args", "-p", "use_sim_time:=false"], output="screen"),
        *supervisor_nodes(),
    ])
