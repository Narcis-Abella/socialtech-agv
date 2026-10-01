"""AMCL-only replay: map_server + AMCL + lifecycle manager on sim time, fed by the /scan, /tf and /Odometry recorded in a full run.
ros2 launch amcl_only.launch.py map:=M.yaml init_x:=X init_y:=Y init_yaw:=R overrides:=extra.yaml   (overrides: overlay.py output, applied after config/amcl.yaml)"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

TOOLS = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
SIM = {"use_sim_time": True}


def number(name):
    return ParameterValue(LC(name), value_type=float)


def generate_launch_description():
    args = [DeclareLaunchArgument(n) for n in ("map", "init_x", "init_y", "init_yaw", "overrides")]
    return LaunchDescription(args + [
        Node(package="nav2_map_server", executable="map_server", parameters=[{"yaml_filename": LC("map")}, SIM]),
        Node(package="nav2_amcl", executable="amcl", parameters=[
            os.path.join(TOOLS, "config/amcl.yaml"), LC("overrides"), SIM,
            {"initial_pose.x": number("init_x"), "initial_pose.y": number("init_y"), "initial_pose.yaw": number("init_yaw")}]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_localization",
             parameters=[{"autostart": True, "node_names": ["map_server", "amcl"]}, SIM]),
    ])
