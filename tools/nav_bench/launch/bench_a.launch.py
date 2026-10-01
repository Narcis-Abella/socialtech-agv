"""Stage A localization bench: FAST-LIO2 odometry + planar base_footprint + deskewed /scan + map_server + AMCL, all on sim time (bag played with --clock).
ros2 launch bench_a.launch.py fastlio_params:=F map:=M.yaml init_x:=X init_y:=Y init_yaw:=R sensor_height:=H level_qx:=.. level_qy:=.. level_qz:=.. level_qw:=..
(level_q* = pose_eval.py level: the rotation odom <- camera_init that levels FAST-LIO2's world frame to the floor of the PGM)"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

TOOLS = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
SIM = {"use_sim_time": True}


def number(name):
    return ParameterValue(LC(name), value_type=float)


def generate_launch_description():
    args = [DeclareLaunchArgument(n) for n in ("fastlio_params", "map", "init_x", "init_y", "init_yaw", "sensor_height", "level_qx", "level_qy", "level_qz", "level_qw")]
    return LaunchDescription(args + [
        Node(package="fast_lio", executable="fastlio_mapping", parameters=[LC("fastlio_params"), SIM]),
        # FAST-LIO2's world frame (camera_init) is the IMU frame at start, not gravity-aligned: odom is that frame levelled to the PGM's floor
        Node(package="tf2_ros", executable="static_transform_publisher", parameters=[SIM],
             arguments=["--frame-id", "odom", "--child-frame-id", "camera_init",
                        "--qx", LC("level_qx"), "--qy", LC("level_qy"), "--qz", LC("level_qz"), "--qw", LC("level_qw")]),
        ExecuteProcess(cmd=["python3", os.path.join(TOOLS, "planar_odom.py"), "--ros-args", "-p", "use_sim_time:=true",
                            "-p", ["sensor_height:=", LC("sensor_height")],
                            "-p", ["level:=[", LC("level_qx"), ",", LC("level_qy"), ",", LC("level_qz"), ",", LC("level_qw"), "]"]], output="screen"),
        Node(package="pointcloud_to_laserscan", executable="pointcloud_to_laserscan_node", parameters=[os.path.join(TOOLS, "config/scan.yaml"), SIM],
             remappings=[("cloud_in", "/cloud_registered_body"), ("scan", "/scan")]),
        Node(package="nav2_map_server", executable="map_server", parameters=[{"yaml_filename": LC("map")}, SIM]),
        Node(package="nav2_amcl", executable="amcl", parameters=[
            os.path.join(TOOLS, "config/amcl.yaml"), SIM,
            {"initial_pose.x": number("init_x"), "initial_pose.y": number("init_y"), "initial_pose.yaw": number("init_yaw")}]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_localization",
             parameters=[{"autostart": True, "node_names": ["map_server", "amcl"]}, SIM]),
    ])
