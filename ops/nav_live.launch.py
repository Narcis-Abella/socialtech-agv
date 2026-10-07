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


def generate_launch_description():
    args = [DeclareLaunchArgument(n) for n in ("livox_config", "fastlio_params", "map", "overrides", "sensor_height", "level_qx", "level_qy", "level_qz", "level_qw")]
    return LaunchDescription(args + [
        Node(package="livox_ros_driver2", executable="livox_ros_driver2_node", name="livox_lidar_publisher", parameters=[{
            "xfer_format": 1, "multi_topic": 0, "data_src": 0, "publish_freq": 10.0, "output_data_type": 0, "frame_id": "livox_frame",
            "lvx_file_path": "/tmp/none.lvx", "user_config_path": LC("livox_config"), "cmdline_input_bd_code": "livox0000000001"}]),
        Node(package="fast_lio", executable="fastlio_mapping", parameters=[LC("fastlio_params")]),
        Node(package="planar_odom", executable="planar_odom", parameters=[{
            "sensor_height": ParameterValue(LC("sensor_height"), value_type=float),
            "level": ParameterValue(["[", LC("level_qx"), ",", LC("level_qy"), ",", LC("level_qz"), ",", LC("level_qw"), "]"], value_type=List[float])}]),
        Node(package="pointcloud_to_laserscan", executable="pointcloud_to_laserscan_node", parameters=[os.path.join(NAV_BENCH, "config/scan.yaml")],
             remappings=[("cloud_in", "/cloud_registered_body"), ("scan", "/scan")]),
        Node(package="nav2_map_server", executable="map_server", parameters=[{"yaml_filename": LC("map")}]),
        Node(package="nav2_amcl", executable="amcl", parameters=[os.path.join(NAV_BENCH, "config/amcl.yaml"), LC("overrides")]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_localization",
             parameters=[{"autostart": True, "node_names": ["map_server", "amcl"]}]),
        # AMCL trail (/amcl_path) and its particles as a PoseArray (/particle_poses), which rviz2 can draw; "-" = no reference path
        ExecuteProcess(cmd=["python3", os.path.join(NAV_BENCH, "live/live_paths.py"), "-", LC("map"), "--ros-args", "-p", "use_sim_time:=false"], output="screen"),
    ])
