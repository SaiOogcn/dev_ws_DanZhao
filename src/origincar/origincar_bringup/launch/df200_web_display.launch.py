from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    launch_args = [
        DeclareLaunchArgument(
            'device',
            default_value=TextSubstitution(text='/dev/video2'),
            description='DF200 usb camera device'
        ),
        DeclareLaunchArgument(
            'image_width',
            default_value=TextSubstitution(text='1920'),
            description='DF200 image width'
        ),
        DeclareLaunchArgument(
            'image_height',
            default_value=TextSubstitution(text='1080'),
            description='DF200 image height'
        ),
        DeclareLaunchArgument(
            'framerate',
            default_value=TextSubstitution(text='30'),
            description='DF200 camera framerate'
        ),
        DeclareLaunchArgument(
            'raw_image_topic',
            default_value=TextSubstitution(text='/image_new'),
            description='DF200 raw image topic for cure_line'
        ),
        DeclareLaunchArgument(
            'mjpeg_image_topic',
            default_value=TextSubstitution(text='/image_new_mjpeg'),
            description='DF200 mjpeg image topic for websocket'
        ),
    ]

    # 1. 启动 DF200，输出 raw 图像 /image_new
    #    这个话题后面可以给 cure_line 使用
    df200_camera_node = Node(
        package='hobot_usb_cam',
        executable='hobot_usb_cam',
        name='df200_camera_node',
        output='screen',
        parameters=[
            {
                'video_device': LaunchConfiguration('device'),
                'pixel_format': 'mjpeg2rgb',
                'image_width': LaunchConfiguration('image_width'),
                'image_height': LaunchConfiguration('image_height'),
                'framerate': LaunchConfiguration('framerate'),
            }
        ],
        remappings=[
            ('/image', LaunchConfiguration('raw_image_topic')),
            ('/camera_info', '/camera_info_df200'),
        ]
    )

    # 2. 把 /image_new 这种 raw rgb8 图像编码成 JPEG/MJPEG
    #    websocket 展示端更适合吃这个压缩图像话题
    df200_codec_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('hobot_codec') + '/launch/hobot_codec_encode.launch.py'
        ),
        launch_arguments={
            'codec_in_mode': 'ros',
            'codec_in_format': 'rgb8',
            'codec_out_mode': 'ros',
            'codec_out_format': 'jpeg',
            'codec_sub_topic': LaunchConfiguration('raw_image_topic'),
            'codec_pub_topic': LaunchConfiguration('mjpeg_image_topic'),
        }.items()
    )

    # 3. websocket 只展示图像，不接 YOLO 检测结果
    #    浏览器展示 /image_new_mjpeg 的真实画面
    df200_web_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('websocket') + '/launch/websocket.launch.py'
        ),
        launch_arguments={
            'websocket_image_topic': LaunchConfiguration('mjpeg_image_topic'),
            'websocket_image_type': 'mjpeg',
            'websocket_only_show_image': 'true',
        }.items()
    )

    return LaunchDescription(launch_args + [
        df200_camera_node,
        df200_codec_node,
        df200_web_node,
    ])

