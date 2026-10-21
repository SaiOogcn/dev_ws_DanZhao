from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    params = os.path.join(get_package_share_directory('demo3_lio'), 'config', 'lio_student.yaml')
    example_path = os.path.join(get_package_share_directory('demo3_lio'), 'config', 'example_path.csv')
    rviz_config = os.path.join(get_package_share_directory('demo3_lio'), 'rviz', 'demo3.rviz')
    return LaunchDescription([
        DeclareLaunchArgument('enable_motion', default_value='true'),
        DeclareLaunchArgument('path_file', default_value=example_path),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='回放带 /clock 的录包时设为 true'),
        Node(package='demo3_lio', executable='lio_student_node', output='screen',
             parameters=[params, {'use_sim_time': ParameterValue(
                 LaunchConfiguration('use_sim_time'), value_type=bool)}]),
        Node(package='demo3_lio', executable='lio_student_controller', output='screen',
             parameters=[params, {'enable_motion': LaunchConfiguration('enable_motion'),
                                  'path_file': LaunchConfiguration('path_file'),
                                  'use_sim_time': ParameterValue(
                                      LaunchConfiguration('use_sim_time'),
                                      value_type=bool)}],
             condition=IfCondition(LaunchConfiguration('enable_motion'))),
        Node(package='rviz2', executable='rviz2', name='demo3_rviz',
             arguments=['-d', rviz_config], output='screen',
             condition=IfCondition(LaunchConfiguration('rviz')))
    ])
