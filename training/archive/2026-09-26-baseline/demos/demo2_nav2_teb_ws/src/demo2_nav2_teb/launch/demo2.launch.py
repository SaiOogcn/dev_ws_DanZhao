from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share = get_package_share_directory('demo2_nav2_teb')
    params = os.path.join(share, 'config', 'nav2_teb_params.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('path_file', default_value=''),
        DeclareLaunchArgument('start_controller', default_value='false'),
        Node(package='demo2_nav2_teb', executable='path_source', output='screen',
             parameters=[{'path_file': LaunchConfiguration('path_file')}]),
        # TODO(trainee): 按车端 Nav2 版本补齐 lifecycle_manager、TF bridge 和 FollowPath client。
        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', output='screen', parameters=[params],
             condition=IfCondition(LaunchConfiguration('start_controller'))),
    ])
