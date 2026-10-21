from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    share = FindPackageShare('demo2_nav2_teb')
    full_params = PathJoinSubstitution([share, 'config', 'nav2_full_params.yaml'])
    example_path = PathJoinSubstitution([share, 'config', 'example_path.csv'])
    map_file = PathJoinSubstitution([share, 'maps', 'training_map.yaml'])
    rviz_config = PathJoinSubstitution([share, 'rviz', 'demo2.rviz'])
    bringup = PathJoinSubstitution([
        FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py'])
    return LaunchDescription([
        DeclareLaunchArgument('path_file', default_value=example_path),
        # 车端录制路径来自 /amcl_pose，坐标系是 map，不是 odom。
        DeclareLaunchArgument('frame_id', default_value='map'),
        # 默认只发路径，不起 Nav2。参数和地图补全后再显式打开。
        DeclareLaunchArgument('start_nav2', default_value='false'),
        DeclareLaunchArgument('map_file', default_value=map_file),
        DeclareLaunchArgument('params_file', default_value=full_params),
        DeclareLaunchArgument('send_goal', default_value='true'),
        # 核心 action 代码完成前的安全门。
        DeclareLaunchArgument('student_ready', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='回放带 /clock 的录包时设为 true'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='启动 RViz2'),
        Node(package='demo2_nav2_teb', executable='path_source', output='screen',
             parameters=[{'path_file': LaunchConfiguration('path_file'),
                          'frame_id': LaunchConfiguration('frame_id'),
                          'use_sim_time': ParameterValue(
                              LaunchConfiguration('use_sim_time'),
                              value_type=bool)}]),
        Node(package='demo2_nav2_teb', executable='follow_path_client', output='screen',
             parameters=[{'send_goal': LaunchConfiguration('send_goal'),
                          'student_ready': LaunchConfiguration('student_ready'),
                          'use_sim_time': ParameterValue(
                              LaunchConfiguration('use_sim_time'),
                              value_type=bool)}]),
        Node(package='rviz2', executable='rviz2', name='demo2_rviz',
             arguments=['-d', rviz_config], output='screen',
             parameters=[{'use_sim_time': ParameterValue(
                 LaunchConfiguration('use_sim_time'), value_type=bool)}],
             condition=IfCondition(LaunchConfiguration('rviz'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup),
            launch_arguments={
                'map': LaunchConfiguration('map_file'),
                'params_file': LaunchConfiguration('params_file'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items(),
            condition=IfCondition(LaunchConfiguration('start_nav2'))),
    ])
