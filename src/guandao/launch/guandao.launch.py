from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share = get_package_share_directory("guandao")
    params = LaunchConfiguration("params_file")
    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(share, "config", "guandao.yaml")),
        DeclareLaunchArgument("path_file", description="Recorded path CSV file"),
        DeclareLaunchArgument("odom_topic", default_value="/odom_combined"),
        DeclareLaunchArgument("vision_topic", default_value="/win_detection"),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("max_forward_speed", default_value="0.60"),
        DeclareLaunchArgument("max_reverse_speed", default_value="0.55"),
        Node(
            package="guandao",
            executable="guandao_node",
            name="guandao_controller",
            output="screen",
            parameters=[
                params,
                {
                    "path_file": LaunchConfiguration("path_file"),
                    "odom_topic": LaunchConfiguration("odom_topic"),
                    "vision_topic": LaunchConfiguration("vision_topic"),
                    "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                    "max_forward_speed": ParameterValue(
                        LaunchConfiguration("max_forward_speed"), value_type=float),
                    "max_reverse_speed": ParameterValue(
                        LaunchConfiguration("max_reverse_speed"), value_type=float),
                },
            ],
        ),
    ])
