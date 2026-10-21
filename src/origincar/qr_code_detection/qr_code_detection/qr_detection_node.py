#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
import cv2
import os
import cv_bridge
import numpy as np
import time
import collections

from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from origincar_msg.msg import Sign
from ai_msgs.msg import PerceptionTargets

from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
)
from ament_index_python.packages import get_package_share_directory

from message_filters import Subscriber, ApproximateTimeSynchronizer

class QrCodeDetectionDebug(Node):
    def __init__(self):
        super().__init__('qrcode_detect_debug')
        self.get_logger().info("启动二维码识别节点（基于二维码框大小2600）")

        self.bridge = cv_bridge.CvBridge()

        # Sign subscribers use the normal ROS 2 reliable/volatile profile.
        # Keep this explicit so the QR handoff does not depend on a distro's
        # implicit QoS defaults (and does not retain stale results).
        sign_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        qos_profile = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_qrcode_info = self.create_publisher(Sign, '/sign_switch', sign_qos)
        self.pub_qr_raw = self.create_publisher(String, '/qr_code_result', qos_profile)

        # 订阅消息杀死自己
        # self.sign_subscriber = self.create_subscription(
        #     Sign,
        #     '/sign_switch',
        #     self.sign_callback,
        #     10
        # )

        self.Signmsg = Sign()
        self.qr_raw_msg = String()

        model_path = os.path.join(get_package_share_directory('qr_code_detection'), 'model/')
        self.get_logger().info(f'Model path: {model_path}')
        self.detect_obj = cv2.wechat_qrcode_WeChatQRCode(
            model_path + 'detect.prototxt', model_path + 'detect.caffemodel',
            model_path + 'sr.prototxt', model_path + 'sr.caffemodel'
        )

        self.scan_finished = False

        # 使用 deque 实现栈（LIFO）
        self.task_stack = collections.deque(maxlen=20)

        self.image_sub = Subscriber(self, CompressedImage, '/image')
        self.detect_sub = Subscriber(self, PerceptionTargets, '/win_detection')

        self.ts = ApproximateTimeSynchronizer(
            [self.image_sub, self.detect_sub],
            queue_size=10,
            slop=0.2
        )
        self.ts.registerCallback(self.synced_callback)

        self.create_timer(0.01, self.worker_loop)

    def is_clear_image(self, image, threshold=350.0):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        lap_var = cv2.Laplacian(gray, cv2.CV_64F).var()
        return lap_var > threshold, lap_var

    def synced_callback(self, image_msg, detect_msg):
        if self.scan_finished:
            return

        has_code = any(t.type == "code" for t in detect_msg.targets)
        if not has_code:
            return

        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(image_msg, desired_encoding='bgr8')

            for target in detect_msg.targets:
                if target.type == "code":
                    for roi in target.rois:
                        w = roi.rect.width
                        h = roi.rect.height
                        area = w * h
                        self.get_logger().info(f"检测到 code 框: 宽={w}, 高={h}, 面积={area}")
                        if area >= 2600:
                            is_clear, lap_var = self.is_clear_image(cv_image)
                            self.get_logger().info(f"图像清晰度（Laplacian方差）: {lap_var:.2f}")
                            if not is_clear:
                                self.get_logger().info("图像模糊，跳过此帧")
                                return

                            # 使用 LIFO 模式，插入栈顶
                            self.task_stack.append((image_msg, detect_msg))
                            self.get_logger().info("二维码框符合大小且图像清晰，已加入处理栈")
                            return
        except Exception as e:
            self.get_logger().error(f"同步回调出错: {e}")

    def worker_loop(self):
        if not self.task_stack or self.scan_finished:
            return

        try:
            # 后进先出
            image_msg, detect_msg = self.task_stack.pop()
            start_time = time.time()

            cv_image = self.bridge.compressed_imgmsg_to_cv2(image_msg, desired_encoding='bgr8')

            for target in detect_msg.targets:
                if target.type == "code":
                    for roi in target.rois:
                        x1 = roi.rect.x_offset
                        y1 = roi.rect.y_offset
                        w = roi.rect.width
                        h = roi.rect.height

                        pad = 25
                        center_x = x1 + w // 2
                        center_y = y1 + h // 2
                        new_w = w + pad * 2
                        new_h = h + pad * 2

                        x1 = max(0, center_x - new_w // 2)
                        y1 = max(0, center_y - new_h // 2)
                        x2 = min(cv_image.shape[1], center_x + new_w // 2)
                        y2 = min(cv_image.shape[0], center_y + new_h // 2)

                        crop = cv_image[y1:y2, x1:x2]

                        timestamp_str = time.strftime("%Y%m%d_%H%M%S", time.localtime())
                        microsec = int((time.time() % 1) * 1e6)
                        filename = f"/userdata/dev_ws/tmp/qr_crop_{timestamp_str}_{microsec:06d}.jpg"
                        cv2.imwrite(filename, crop)
                        self.get_logger().info(f"截图已保存：{filename}")

                        qr_info, _ = self.detect_obj.detectAndDecode(crop)
                        if qr_info and qr_info[0].strip():
                            end_time = time.time()
                            duration = end_time - start_time
                            qr_text = qr_info[0].strip()
                            self.get_logger().info(f"识别出二维码: {qr_text}")
                            self.get_logger().info(f"扫码成功，耗时: {duration:.3f} 秒")
                            self.get_logger().info(f"成功的照片：{filename}")

                            self.qr_raw_msg.data = qr_text
                            self.pub_qr_raw.publish(self.qr_raw_msg)

                            if qr_text[-1].isdigit():
                                last_digit = int(qr_text[-1])
                                self.Signmsg.sign_data = 3 if last_digit % 2 == 1 else 4
                                self.pub_qrcode_info.publish(self.Signmsg)
                                self.get_logger().info(f"已发布 sign_data: {self.Signmsg.sign_data}")
                                # self.shutdown_node()
                                return

        except Exception as e:
            self.get_logger().error(f"后台处理异常: {e}")

    # def sign_callback(self, msg: Sign):
    #     if msg.sign_data in [3, 4] and not self.scan_finished:
    #         self.get_logger().info(f"另一个节点扫到了，（sign_data={msg.sign_data}，准备关闭节点")
    #         self.shutdown_node()

    def shutdown_node(self):
        self.get_logger().info("扫码成功，节点关闭")
        self.scan_finished = True
        self.task_stack.clear()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = QrCodeDetectionDebug()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
