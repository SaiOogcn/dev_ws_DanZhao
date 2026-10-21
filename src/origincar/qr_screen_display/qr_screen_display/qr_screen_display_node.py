#!/usr/bin/env python3

import os
import re

import cv2
import numpy as np
import rclpy
from PIL import Image, ImageDraw, ImageFont
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import String
from origincar_msg.msg import Sign


class QRScreenDisplay(Node):
    """Render QR data, direction, and a future Chinese caption on the HDMI screen."""

    def __init__(self):
        super().__init__('qr_screen_display')

        self.qr_topic = self.declare_parameter(
            'qr_result_topic', '/qr_code_result').value
        self.direction_topic = self.declare_parameter(
            'direction_topic', '/sign_switch').value
        # take_photo/rdk_client can publish its generated Chinese description here later.
        self.caption_topic = self.declare_parameter(
            'caption_topic', '/take_photo/result').value
        self.font_path = self.declare_parameter('font_path', '').value
        self.width = int(self.declare_parameter('width', 1280).value)
        self.height = int(self.declare_parameter('height', 720).value)
        fullscreen_value = self.declare_parameter('fullscreen', True).value
        if isinstance(fullscreen_value, str):
            self.fullscreen = fullscreen_value.lower() in ('1', 'true', 'yes', 'on')
        else:
            self.fullscreen = bool(fullscreen_value)

        self.qr_text = '等待扫码...'
        self.direction = '等待方向...'
        self.caption = '等待图生文结果...'

        # Raw QR text and the PC caption are retained. /sign_switch is deliberately
        # volatile so a stale direction cannot start a new race mission.
        latched_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        direction_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self.create_subscription(String, self.qr_topic, self.qr_callback, latched_qos)
        self.create_subscription(Sign, self.direction_topic,
                                 self.direction_callback, direction_qos)
        self.create_subscription(String, self.caption_topic,
                                 self.caption_callback, latched_qos)

        self.font_file = self._find_font(self.font_path)
        if self.font_file:
            self.get_logger().info(f'使用中文字体: {self.font_file}')
        else:
            self.get_logger().warning(
                '没有找到中文字体。请安装 fonts-noto-cjk 或通过参数 font_path 指定字体文件。')

        self.title_font = self._load_font(48)
        self.label_font = self._load_font(34)
        self.value_font = self._load_font(56)
        self.caption_font = self._load_font(32)

        self.gui_available = True
        try:
            cv2.namedWindow('QR Code Result', cv2.WINDOW_NORMAL)
            if self.fullscreen:
                cv2.setWindowProperty(
                    'QR Code Result', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
            else:
                cv2.resizeWindow('QR Code Result', self.width, self.height)
        except cv2.error as error:
            self.gui_available = False
            self.get_logger().error(
                f'无法创建屏幕窗口，请确认板子有桌面显示会话和 DISPLAY 环境变量: {error}')

        self.timer = self.create_timer(0.05, self.render)
        self.get_logger().info(
            f'订阅 QR={self.qr_topic}, direction={self.direction_topic}, '
            f'caption={self.caption_topic}')

    @staticmethod
    def _find_font(requested):
        candidates = []
        if requested:
            candidates.append(requested)
        candidates.extend([
            '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
            '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.otf',
            '/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc',
            '/usr/share/fonts/truetype/wqy/wqy-microhei.ttc',
            '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',
        ])
        for path in candidates:
            if os.path.isfile(path):
                return path
        return None

    def _load_font(self, size):
        if self.font_file:
            try:
                return ImageFont.truetype(self.font_file, size)
            except OSError as error:
                self.get_logger().warning(f'加载字体失败 {self.font_file}: {error}')
        return ImageFont.load_default()

    def qr_callback(self, msg):
        value = msg.data.strip()
        if not value:
            return

        # The current scanner sends a four-digit code. Keep unexpected values visible
        # instead of silently discarding them, which makes diagnosis easier.
        if re.fullmatch(r'\d{4}', value):
            # Competition rule: odd last digit means clockwise, even means
            # counter-clockwise. Derive this locally so the display does not
            # depend on the timing of the /sign_switch message.
            self.direction = '顺时针' if int(value[-1]) % 2 == 1 else '逆时针'
        else:
            self.get_logger().warning(f'扫码内容不是四位数字，将原样显示: {value}')
        self.qr_text = value

        # Also support the older detector that sends the direction as the QR text.
        normalized = value.lower().replace('_', '').replace('-', '')
        if normalized in ('clockwise', 'shunshizhen'):
            self.direction = '顺时针'
        elif normalized in ('anticlockwise', 'counterclockwise', 'nishizhen'):
            self.direction = '逆时针'

    def direction_callback(self, msg):
        if msg.sign_data == 3:
            self.direction = '顺时针'
        elif msg.sign_data == 4:
            self.direction = '逆时针'
        else:
            self.direction = f'未知 ({msg.sign_data})'

    def caption_callback(self, msg):
        text = msg.data.strip()
        if text:
            self.caption = text

    @staticmethod
    def _wrap_text(draw, text, font, max_width):
        lines = []
        for paragraph in text.splitlines() or ['']:
            current = ''
            for char in paragraph:
                candidate = current + char
                box = draw.textbbox((0, 0), candidate, font=font)
                if current and box[2] > max_width:
                    lines.append(current)
                    current = char
                else:
                    current = candidate
            lines.append(current)
        return lines or ['']

    def render(self):
        if not self.gui_available:
            return

        canvas = Image.new('RGB', (self.width, self.height), (12, 20, 35))
        draw = ImageDraw.Draw(canvas)

        draw.text((50, 35), '扫码结果', font=self.title_font, fill=(90, 220, 255))
        draw.text((70, 130), '四位数字：', font=self.label_font, fill=(220, 220, 220))
        draw.text((330, 120), self.qr_text, font=self.value_font, fill=(255, 255, 255))
        draw.text((70, 225), '方向：', font=self.label_font, fill=(220, 220, 220))
        direction_color = (80, 255, 120) if self.direction in ('顺时针', '逆时针') else (255, 200, 80)
        draw.text((230, 215), self.direction, font=self.value_font, fill=direction_color)

        draw.line((50, 320, self.width - 50, 320), fill=(80, 100, 120), width=2)
        draw.text((50, 345), '图生文', font=self.label_font, fill=(90, 220, 255))

        caption_lines = self._wrap_text(
            draw, self.caption, self.caption_font, self.width - 100)
        y = 415
        for line in caption_lines[:6]:
            draw.text((70, y), line, font=self.caption_font, fill=(255, 255, 255))
            y += 45

        frame = cv2.cvtColor(np.asarray(canvas), cv2.COLOR_RGB2BGR)
        cv2.imshow('QR Code Result', frame)
        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord('q')):
            rclpy.shutdown()

    def destroy_node(self):
        if self.gui_available:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = QRScreenDisplay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
