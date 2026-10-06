"""Nav2 on the simulated Tracer: localization (AMCL on the PGM) + navigation. The simulator is launched separately."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory("socialtech_nav")
    bringup = get_package_share_directory("nav2_bringup")
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=os.path.join(share, "config", "nav2_params.yaml")),
        DeclareLaunchArgument("map", default_value=os.path.join(share, "maps", "loyola.yaml")),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, "launch", "bringup_launch.py")),
            launch_arguments={
                "params_file": LaunchConfiguration("params_file"),
                "map": LaunchConfiguration("map"),
                "use_sim_time": "true",
                "autostart": "true",
                "use_composition": "False",
            }.items(),
        ),
    ])
