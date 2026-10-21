from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    params = os.path.join(get_package_share_directory('demo3_lio'), 'config', 'lio_student.yaml')
    return LaunchDescription([
        Node(package='demo3_lio', executable='lio_student_node', output='screen',
             parameters=[params])
    ])
