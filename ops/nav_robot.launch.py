"""Navigation on the real robot: the live localization (nav_live.launch.py: driver, FAST-LIO2, odom, scan, map_server, AMCL) + Nav2's navigation_launch.py
(planner, controller, smoothers, behaviors, collision monitor) + the Tracer base only with base:=true (without it a goal moves nothing). Same arguments as
nav_live.launch.py plus nav2_params:=P.yaml.
The base publishes its own wheel odometry TF: its frames are renamed so they never collide with planar_odom's odom -> base_footprint; its /odom topic gives Nav2 the real speed."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as LC

HERE = os.path.dirname(os.path.realpath(__file__))
LIVE_ARGS = ("livox_config", "fastlio_params", "map", "overrides", "sensor_height", "level_qx", "level_qy", "level_qz", "level_qw")


def include(path, condition=None, **args):
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(path), launch_arguments=args.items(), condition=condition)


def generate_launch_description():
    nav2 = get_package_share_directory("nav2_bringup")
    tracer = get_package_share_directory("tracer_base")
    return LaunchDescription([DeclareLaunchArgument(n) for n in LIVE_ARGS + ("nav2_params",)] + [DeclareLaunchArgument("base", default_value="false")] + [
        include(os.path.join(HERE, "nav_live.launch.py"), **{n: LC(n) for n in LIVE_ARGS}),
        include(os.path.join(nav2, "launch", "navigation_launch.py"), params_file=LC("nav2_params"), use_sim_time="false", autostart="false", use_composition="False"),   # started by nav_start_when_localized.py
        include(os.path.join(tracer, "launch", "tracer_base.launch.py"), IfCondition(LC("base")), port_name="can_tracer", odom_frame="wheel_odom", base_frame="wheel_base_link"),
    ])
