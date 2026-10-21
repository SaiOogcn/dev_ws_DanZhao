import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from origincar_msg.msg import Sign  # 自定义消息类型
from pyzbar.pyzbar import decode
from PIL import Image
import numpy as np
import cv2
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from std_msgs.msg import String


class QRCodeScanner(Node):
    def __init__(self):
        super().__init__('qr_code_scanner')

        # 订阅压缩图像消息
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image',
            self.listener_callback,
            10
        )

        # 设置 QoS，用于上位机接收数据
        qos_profile = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        # 发布控制信号（奇偶）
        self.pub_qrcode_info = self.create_publisher(Sign, '/sign_switch', 10)

        # 发布原始二维码数据（给上位机）
        self.pub_qr_raw = self.create_publisher(String, '/qr_code_result', qos_profile)

        self.Signmsg = Sign()
        self.detected_qrcodes = set()

        self.get_logger().info("二维码扫描节点已启动。")

    def listener_callback(self, msg: CompressedImage):
        try:
            # 解压缩图像
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            pil_image = Image.fromarray(gray)

            # 使用 pyzbar 解码
            decoded_objects = decode(pil_image)

            for obj in decoded_objects:
                qr_data = obj.data.decode("utf-8").strip()
                if qr_data in self.detected_qrcodes:
                    continue

                self.detected_qrcodes.add(qr_data)
                self.get_logger().info(f"扫描到二维码内容: {qr_data}")

                # -------- 合法性校验开始 --------
                if not qr_data.isdigit():
                    self.get_logger().warn("二维码内容不是纯数字，忽略。")
                    continue

                qr_number = int(qr_data)

                if not (0 <= qr_number < 10000):
                    self.get_logger().warn(f"数字 {qr_number} 不在 0~9999 范围内，忽略。")
                    continue
                # -------- 合法性校验结束 --------

                # 奇偶判断并发布
                if qr_number % 2 == 0:
                    self.Signmsg.sign_data = 4
                    self.get_logger().info(f"数值为偶数: {qr_number}，发布 4")
                else:
                    self.Signmsg.sign_data = 3
                    self.get_logger().info(f"数值为奇数: {qr_number}，发布 3")

                # 向上位机发布原始内容
                self.pub_qr_raw.publish(String(data=qr_data))
                self.get_logger().info("已向上位机发布二维码原始内容")

                self.pub_qrcode_info.publish(self.Signmsg)
                # self.shutdown_node()  # 发布成功后关闭节点

        except Exception as e:
            self.get_logger().error(f"处理图像时出错: {e}")

    def shutdown_node(self):
        self.get_logger().info('扫码成功，节点即将关闭。')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = QRCodeScanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
