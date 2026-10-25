from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    default_config = os.path.join(get_package_share_directory('zsh_navigation'),
                                  'config', 'navigation.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        DeclareLaunchArgument('direction', default_value='CW'),
        Node(package='zsh_navigation', executable='full_race_navigation',
             name='zsh_navigation', output='screen',
             parameters=[LaunchConfiguration('config'), {
                 'direction': ParameterValue(LaunchConfiguration('direction'), value_type=str),
             }]),
    ])
