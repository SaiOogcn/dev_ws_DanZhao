from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(get_package_share_directory(
        'demo1_visual_control'), 'config', 'visual_controller.yaml')
    return LaunchDescription([
        Node(package='demo1_visual_control', executable='visual_controller',
             name='demo1_visual_controller', output='screen',
             parameters=[ParameterFile(config, allow_substs=True)])
    ])
