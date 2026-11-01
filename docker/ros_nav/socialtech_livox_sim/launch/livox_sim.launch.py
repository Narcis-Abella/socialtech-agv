from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node


def generate_launch_description():
    args = {"model_file": "", "scene_file": "", "pose_topic": "/odom_gt_raw", "custom_topic": "/livox/lidar", "pc2_topic": "", "frame_id": "livox_mid360_link"}
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in args.items()] + [
        Node(package="socialtech_livox_sim", executable="livox_mid360_sim", output="screen",
             parameters=[{"model_file": LC("model_file"), "scene_file": LC("scene_file"), "pose_topic": LC("pose_topic"), "custom_topic": LC("custom_topic"),
                          "pc2_topic": LC("pc2_topic"), "frame_id": LC("frame_id")}])])
