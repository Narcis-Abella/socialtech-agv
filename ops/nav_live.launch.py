"""Live localization on the robot: Livox driver + FAST-LIO2 + levelled planar odom + /scan + map_server + gn_loc, on wall time. gn_loc gets no initial pose here: from the supervisor, or click 2D Pose Estimate in rviz2.
ros2 launch nav_live.launch.py livox_config:=C.json fastlio_params:=F map:=M.yaml sensor_height:=H level_qx:=.. level_qy:=.. level_qz:=.. level_qw:=.. overrides:=O.yaml
driver:=false sim_time:=true: the simulation (tools/nav_sim/sup_sim.sh) publishes /livox/lidar and /livox/imu itself and runs on /clock; everything else is this same launch."""
import os
from typing import List

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration as LC, PythonExpression
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue

NAV_COMMON = os.path.join(os.path.dirname(os.path.dirname(os.path.realpath(__file__))), "tools/nav_common")


SCAN = os.path.join(NAV_COMMON, "config/scan.yaml")
CLOUD = [("cloud_in", "/cloud_registered_body")]
LEVEL_MAP_DIR = os.environ.get("LEVEL_MAP_DIR", "/map")   # where map.ply + map.yaml are: the live stack mounts the map at /map, the sim writes its own
LEVEL = ParameterValue(["[", LC("level_qx"), ",", LC("level_qy"), ",", LC("level_qz"), ",", LC("level_qw"), "]"], value_type=List[float])   # odom <- camera_init
LIDAR_IN_BODY = [-0.011, -0.02329, 0.04412]   # extrinsic_T of FAST-LIO2's config (upstream mid360.yaml, unchanged by nav_live.sh)


def above_lidar(height):
    """A height above the floor (scan.yaml) as seen from lidar_level, whose height is that of the map's sensor_height."""
    return ParameterValue(PythonExpression([str(height), " - (", LC("sensor_height"), f" + {LIDAR_IN_BODY[2]})"]), value_type=float)


def scan_nodes():
    """/scan from FAST-LIO2's cloud: the stock pointcloud_to_laserscan (the closest point per bin). SCAN_TOPIC (default /scan): where it publishes (the simulation puts a noise model between it and /scan).
    In lidar_level, the height of the sensor: on the floor it would sit on the costmaps' min_obstacle_height 0, and the ~3 cm of error of map -> odom's z would drop half of the scans (48 % with AMCL in bag 162508)."""
    topic = os.environ.get("SCAN_TOPIC", "/scan")
    with open(SCAN) as f:
        band = yaml.safe_load(f)["/**"]["ros__parameters"]
    return [Node(package="pointcloud_to_laserscan", executable="pointcloud_to_laserscan_node", remappings=CLOUD + [("scan", topic)],
                 parameters=[SCAN, {"target_frame": "lidar_level", "min_height": above_lidar(band["min_height"]), "max_height": above_lidar(band["max_height"])}])]


def localizer_nodes(overrides):
    """gn_loc (Gauss-Newton scan-to-map, built in the image): /initialpose in, map->odom and gn_loc/pose out."""
    return [Node(package="gn_loc", executable="gn_loc_node", name="gn_loc", output="screen", parameters=[overrides]),
            Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_localization",
                 parameters=[{"autostart": True, "node_names": ["map_server", "gn_loc"]}])]


HEAVY_URI = ('<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces><MaxMessageSize>65500B</MaxMessageSize><FragmentSize>64000B</FragmentSize></General>'
             '<Discovery><Peers><Peer address="localhost"/></Peers><MaxAutoParticipantIndex>30</MaxAutoParticipantIndex></Discovery><Internal><SocketReceiveBufferSize min="10485760B"/></Internal></Domain></CycloneDDS>')


def heavy_env():
    """HEAVY_DDS=1: the driver and FAST-LIO2 (the two ends of the 400 kB x 10 Hz lidar topic) talk on loopback with 64 kB datagrams instead of 1344 B fragments: ~45 % less transport CPU on
    that hop and 73 % fewer sendmsg (laptop microbenchmark, notes/optimizacion_localizacion_2026-11-02.md; NOT measured on the Nano, hence off by default). A reader on the default URI would have to receive them across that crossing, which is NOT tested: nav_live.sh refuses HEAVY_DDS with SUPERVISOR (the supervisor reads the raw lidar and IMU); check `ros2 topic hz /livox/lidar` from a default-URI node before lifting that.
    Their topics and FAST-LIO2's TF are then not visible from the laptop's rviz2 (the configs in tools/ do not use them). The rcvbuf line is explicit: the entrypoint never sees a per-node env."""
    return {"additional_env": {"CYCLONEDDS_URI": HEAVY_URI}} if os.environ.get("HEAVY_DDS", "").lower() not in ("", "0", "false", "no", "off") else {}


def supervisor_nodes():
    """SUPERVISOR=1 (ops/nav_live.sh): gn_supervisor_node.py staged in /greloc. Respawned: a restarted supervisor comes up in CONFIRMING and holds Nav2 until
    gn_loc confirms its pose. It reads the robot's own FAST-LIO2 (/Odometry for "is the restarted filter publishing", plus the raw /livox/lidar and /livox/imu): no second FAST-LIO, unlike the AMCL-era reloc_node (PR #14, removed 2026-11-08)."""
    if os.environ.get("SUPERVISOR", "").lower() in ("", "0", "false", "no", "off"):    # the rule of ops/flags.sh
        return []
    prior = os.environ.get("SUP_PRIOR")
    if not prior:
        raise RuntimeError("SUPERVISOR is set but SUP_PRIOR (the 3D prior of the map) is not: start it with ops/nav_live.sh, which checks the staged files and sets it")
    return [ExecuteProcess(name="gn_supervisor", output="screen", respawn=True, respawn_delay=2.0, cmd=[
        "python3", "-B", "/greloc/tools/global_reloc_3d/gn_supervisor_node.py", "--ros-args",
        "-p", f"map_xyz:={prior}", "-p", "register_binary:=/greloc/build/register_v2", "-p", "lib_dir:=/greloc/build",
        "-p", "wheel_topic:=/odom", "-p", "work_dir:=/runs/gn_supervisor", "-p", f"base_from_body_xyyaw:=[{LIDAR_IN_BODY[0]}, {LIDAR_IN_BODY[1]}, 0.0]",
        "-p", ["sensor_z:=", LC("sensor_height")],
        "-p", f"lidar_decimate:={int(os.environ.get('POINT_FILTER_NUM') or 2)}"])]      # the density FAST-LIO2 runs with (nav_live.sh point_filter_num); the search was validated with 2, so another value is a change to re-validate     # the mounting height over the floor of the map: the relocalizer searches only +- 0.5 m around it


def generate_launch_description():
    args = [DeclareLaunchArgument(n) for n in ("livox_config", "fastlio_params", "map", "overrides", "sensor_height", "level_qx", "level_qy", "level_qz", "level_qw")]
    args += [DeclareLaunchArgument("driver", default_value="true"), DeclareLaunchArgument("sim_time", default_value="false"), DeclareLaunchArgument("viz", default_value="true"), DeclareLaunchArgument("level_corrector", default_value="false")]
    return LaunchDescription(args + [
        SetParameter(name="use_sim_time", value=True, condition=IfCondition(LC("sim_time"))),
        Node(package="livox_ros_driver2", executable="livox_ros_driver2_node", name="livox_lidar_publisher", parameters=[{
            "xfer_format": 1, "multi_topic": 0, "data_src": 0, "publish_freq": 10.0, "output_data_type": 0, "frame_id": "livox_frame",
            "lvx_file_path": "/tmp/none.lvx", "user_config_path": LC("livox_config"), "cmdline_input_bd_code": "livox0000000001"}],
            condition=IfCondition(LC("driver")), **heavy_env()),
        # respawn: gn_supervisor restarts it by killing it (a fresh filter takes gravity from a still IMU and starts at z = 0, which a long blind period does not leave)
        Node(package="fast_lio", executable="fastlio_mapping", parameters=[LC("fastlio_params")], respawn=True, respawn_delay=1.0, **heavy_env()),
        # level_corrector:=true (nav_live.sh LEVEL_CORRECTOR=1): planar_odom takes the roll/pitch correction of the odom frame that level_corrector registers against the map; off: the static level, as always
        Node(package="planar_odom", executable="planar_odom", parameters=[{
            "sensor_height": ParameterValue(LC("sensor_height"), value_type=float),
            "lidar_in_body": LIDAR_IN_BODY,
            "level": LEVEL,
            "correction_topic": ParameterValue(PythonExpression(["'/level_corrector/correction' if '", LC("level_corrector"), "' == 'true' else ''"]), value_type=str)}]),
        Node(package="level_corrector", executable="level_corrector_node", parameters=[{"map_dir": LEVEL_MAP_DIR, "level": LEVEL}], condition=IfCondition(LC("level_corrector")), respawn=True, respawn_delay=2.0),
        *scan_nodes(),
        Node(package="nav2_map_server", executable="map_server", parameters=[{"yaml_filename": LC("map")}, LC("overrides")]),
        *localizer_nodes(LC("overrides")),
        # gn_loc's trail (/gn_path); "-" = no reference path. A rviz aid: viz:=false (PROD=1) leaves it out
        ExecuteProcess(cmd=["python3", os.path.join(NAV_COMMON, "live/live_paths.py"), "-", LC("map"), "--ros-args", "-p", "use_sim_time:=false"], output="screen", condition=IfCondition(LC("viz"))),
        ExecuteProcess(name="local_frame_aligner", output="screen", respawn=True, respawn_delay=2.0, cmd=["python3", "-B", os.path.join(os.path.dirname(os.path.realpath(__file__)), "local_frame_aligner.py"), "--ros-args", "-p", ["use_sim_time:=", LC("sim_time")]]),
        *supervisor_nodes(),
    ])
