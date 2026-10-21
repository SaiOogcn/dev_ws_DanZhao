from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
rviz_path=get_package_share_directory('deptrum-ros-driver-aurora930')

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="rviz2",
            executable="rviz2",
            name='rviz2',
            parameters=[
            ],
            arguments=['-d', rviz_path+'/rviz/aurora930-ros2.rviz'],
            output="screen",   
        )
    ])