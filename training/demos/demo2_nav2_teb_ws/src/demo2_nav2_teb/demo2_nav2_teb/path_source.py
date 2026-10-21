import csv
import math
import os

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node


class PathSource(Node):
    """录制路径发布节点。

    支持两种 CSV 格式：

    1. 三列 `x,y,yaw`，yaw 单位是弧度，教学用。
    2. 车端保存器写出的七列 `x,y,z,qx,qy,qz,qw`，行尾带一个逗号；
       后面可以再跟第 8 列行驶方向，`1` 前进，`-1` 倒车。

    `dir` 列会被读出来并统计，但这里不参与发号。需要倒车段时由控制器处理。
    路径坐标系由 `frame_id` 决定；车端录制路径来自 `/amcl_pose`，坐标系是 `map`。
    """

    def __init__(self):
        super().__init__('demo2_path_source')
        self.declare_parameter('path_file', '')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_period_s', 1.0)
        self.pub = self.create_publisher(Path, '/demo2/reference_path', 1)
        self.timer = self.create_timer(
            self.get_parameter('publish_period_s').value, self.publish_path)
        self.reported_ = False

    def parse_row(self, row, header):
        """把一行 CSV 变成 PoseStamped。格式不对时返回 None。"""
        fields = [field.strip() for field in row]
        while fields and fields[-1] == '':
            fields.pop()
        try:
            if len(fields) >= 7:
                x, y, z = float(fields[0]), float(fields[1]), float(fields[2])
                qx, qy, qz, qw = (float(fields[3]), float(fields[4]),
                                  float(fields[5]), float(fields[6]))
                direction = int(float(fields[7])) if len(fields) >= 8 else 1
            elif len(fields) >= 3:
                x, y = float(fields[0]), float(fields[1])
                yaw = float(fields[2])
                z, qx, qy = 0.0, 0.0, 0.0
                qz, qw = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
                direction = 1
            else:
                return None, 0
        except (TypeError, ValueError):
            return None, 0

        pose = PoseStamped()
        pose.header = header
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose, direction

    def publish_path(self):
        filename = self.get_parameter('path_file').value
        frame_id = self.get_parameter('frame_id').value
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = frame_id
        if not filename or not os.path.exists(filename):
            self.get_logger().warn('path_file 不存在：请补全录制路径配置')
            self.pub.publish(path)
            return

        # TODO(学员): 检查点间距、重复点和跳变点，并决定是否需要平滑与重采样。
        # 这里的 frame_id 必须和路径实际录制时的坐标系一致，车端录制路径是 map。
        bad_rows = 0
        reverse_points = 0
        with open(filename, newline='', encoding='utf-8') as handle:
            for row in csv.reader(handle):
                if not row or row[0].lstrip().startswith('#'):
                    continue
                pose, direction = self.parse_row(row, path.header)
                if pose is None:
                    bad_rows += 1
                    continue
                if direction < 0:
                    reverse_points += 1
                path.poses.append(pose)

        if not self.reported_:
            self.reported_ = True
            self.get_logger().info(
                '路径载入完成：%d 个位姿点，%d 行格式不符，%d 个倒车点，frame_id=%s'
                % (len(path.poses), bad_rows, reverse_points, frame_id))
            if reverse_points:
                self.get_logger().warn(
                    '路径含倒车点，本节点不处理第 8 列方向，需要控制器自己判断')
        self.pub.publish(path)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(PathSource())
    rclpy.shutdown()
