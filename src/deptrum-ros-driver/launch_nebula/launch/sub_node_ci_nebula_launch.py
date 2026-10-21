from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="deptrum-ros-driver-nebula",
            executable="sub_node_ci",
            namespace="nebula",
            parameters=[
                {"enable_rgb": LaunchConfiguration('enable_rgb', default=True),
                 "enable_ir" : LaunchConfiguration('enable_ir', default=True),
                 "enable_depth" : LaunchConfiguration('enable_depth', default=True),
                 "enable_pointcloud" : LaunchConfiguration('enable_pointcloud', default=True),
                 "enable_stof_rgb": LaunchConfiguration('enable_stof_rgb', default=True),
                 "enable_flag" : LaunchConfiguration('enable_flag', default=True),
                 "enable_stof_depth" : LaunchConfiguration('enable_stof_depth', default=True),
                 "enable_stof_pointcloud" : LaunchConfiguration('enable_stof_pointcloud', default=True),
                 }
            ],
            arguments=None,
            output="screen",   
        )
    ])