import csv
import math
import os

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node


class PathSource(Node):
    """发布录制路径，供 Nav2 FollowPath 或 dzfollow_path 适配器使用。"""

    def __init__(self):
        super().__init__('demo2_path_source')
        self.declare_parameter('path_file', '')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_period_s', 1.0)
        self.pub = self.create_publisher(Path, '/demo2/reference_path', 1)
        self.timer = self.create_timer(
            self.get_parameter('publish_period_s').value, self.publish_path)

    def publish_path(self):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.get_parameter('frame_id').value
        filename = self.get_parameter('path_file').value
        if not filename or not os.path.exists(filename):
            self.get_logger().warn('path_file 不存在，等待学员配置录制路径')
            self.pub.publish(path)
            return
        with open(filename, newline='', encoding='utf-8') as handle:
            for row in csv.reader(handle):
                if len(row) < 3:
                    continue
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x = float(row[0])
                pose.pose.position.y = float(row[1])
                yaw = float(row[2])
                pose.pose.orientation.z = math.sin(yaw / 2.0)
                pose.pose.orientation.w = math.cos(yaw / 2.0)
                path.poses.append(pose)
        self.pub.publish(path)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(PathSource())
    rclpy.shutdown()
