#!/usr/bin/env python3

import os
import queue
import threading
import time
import uuid

import cv2
import numpy as np
import rclpy
import zmq
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String


class RingPhotoClient(Node):
    """Capture one requested camera frame and exchange it with the PC server."""

    def __init__(self):
        super().__init__('ring_photo_client')

        self.trigger_topic = self.declare_parameter(
            'trigger_topic', '/take_photo/trigger').value
        self.result_topic = self.declare_parameter(
            'result_topic', '/take_photo/result').value
        self.status_topic = self.declare_parameter(
            'status_topic', '/take_photo/status').value
        self.front_image_topic = self.declare_parameter(
            'front_image_topic', '/image').value
        self.aurora_image_topic = self.declare_parameter(
            'aurora_image_topic', '/aurora/rgb/image_raw').value
        self.server_ip = self.declare_parameter(
            'server_ip', '172.20.10.9').value
        self.server_port = int(self.declare_parameter('server_port', 5555).value)
        self.output_width = max(
            1, int(self.declare_parameter('output_width', 640).value))
        self.output_height = max(
            1, int(self.declare_parameter('output_height', 400).value))
        self.jpeg_quality = min(
            100, max(1, int(self.declare_parameter('jpeg_quality', 100).value)))
        self.camera_wait_timeout = max(
            0.0, float(self.declare_parameter('camera_wait_timeout', 2.0).value))
        self.camera_frame_max_age = max(
            0.0, float(self.declare_parameter('camera_frame_max_age', 1.0).value))
        self.debug_image_path = self.declare_parameter(
            'debug_image_path', '/tmp/ring_photo_latest.jpg').value

        self.upload_timeout_ms = max(
            100, int(self.declare_parameter('upload_timeout_ms', 10000).value))
        self.poll_timeout_ms = max(
            100, int(self.declare_parameter('poll_timeout_ms', 5000).value))
        self.request_retries = max(
            1, int(self.declare_parameter('request_retries', 10).value))
        self.result_poll_initial_delay = max(
            0.0, float(self.declare_parameter(
                'result_poll_initial_delay', 4.0).value))
        self.result_poll_interval = max(
            0.05, float(self.declare_parameter('result_poll_interval', 1.0).value))
        self.result_poll_max_interval = max(
            self.result_poll_interval,
            float(self.declare_parameter('result_poll_max_interval', 5.0).value),
        )
        self.result_poll_retries = max(
            1, int(self.declare_parameter('result_poll_retries', 180).value))

        trigger_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        latched_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.result_pub = self.create_publisher(String, self.result_topic, latched_qos)
        self.status_pub = self.create_publisher(String, self.status_topic, latched_qos)
        self.create_subscription(
            String, self.trigger_topic, self._trigger_callback, trigger_qos)
        self.create_subscription(
            CompressedImage,
            self.front_image_topic,
            self._front_image_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            self.aurora_image_topic,
            self._aurora_image_callback,
            qos_profile_sensor_data,
        )

        self._frame_lock = threading.Lock()
        self._front_frame = None
        self._front_frame_time = 0.0
        self._aurora_frame = None
        self._aurora_frame_time = 0.0
        self._request_active = False
        self._jobs = queue.Queue(maxsize=1)
        self._events = queue.Queue()
        self._stop_event = threading.Event()
        self._worker = threading.Thread(
            target=self._worker_loop, name='ring-photo-zmq', daemon=True)
        self._worker.start()
        self._event_timer = self.create_timer(0.10, self._publish_worker_events)

        self.get_logger().info(
            '环形区拍照客户端已启动：触发=%s 前相机=%s Aurora=%s PC=tcp://%s:%d' % (
                self.trigger_topic,
                self.front_image_topic,
                self.aurora_image_topic,
                self.server_ip,
                self.server_port,
            )
        )

    def _front_image_callback(self, msg):
        with self._frame_lock:
            self._front_frame = msg
            self._front_frame_time = time.monotonic()

    def _aurora_image_callback(self, msg):
        with self._frame_lock:
            self._aurora_frame = msg
            self._aurora_frame_time = time.monotonic()

    def _trigger_callback(self, msg):
        camera = msg.data.strip().lower()
        aliases = {
            'front': 'front',
            'yolo': 'front',
            'forward': 'front',
            'aurora': 'aurora',
            'rear': 'aurora',
            'reverse': 'aurora',
        }
        camera = aliases.get(camera)
        if camera is None:
            self.get_logger().warning(
                f'忽略未知拍照触发内容: {msg.data!r}，应为 front 或 aurora')
            return

        with self._frame_lock:
            if self._request_active:
                self.get_logger().warning('已有图片正在上传/推理，忽略重复拍照触发')
                return
            self._request_active = True

        try:
            self._jobs.put_nowait(camera)
        except queue.Full:
            with self._frame_lock:
                self._request_active = False
            self.get_logger().warning('拍照任务队列已满，忽略本次触发')
            return

        camera_name = '车头YOLO相机' if camera == 'front' else '车尾Aurora相机'
        self._queue_status(f'已触发拍照，等待{camera_name}最新画面')
        self.get_logger().warning(f'收到环形区拍照触发：{camera_name}')

    def _latest_frame(self, camera):
        with self._frame_lock:
            if camera == 'front':
                return self._front_frame, self._front_frame_time
            return self._aurora_frame, self._aurora_frame_time

    def _wait_for_current_frame(self, camera):
        deadline = time.monotonic() + self.camera_wait_timeout
        while not self._stop_event.is_set():
            frame, received_at = self._latest_frame(camera)
            age = time.monotonic() - received_at
            if frame is not None and (
                self.camera_frame_max_age <= 0.0 or age <= self.camera_frame_max_age
            ):
                return frame
            if time.monotonic() >= deadline:
                return None
            self._stop_event.wait(0.05)
        return None

    @staticmethod
    def _decode_compressed(msg):
        encoded = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError('OpenCV无法解码前置相机压缩图像')
        return image

    @staticmethod
    def _decode_raw(msg):
        encoding = msg.encoding.lower()
        height = int(msg.height)
        width = int(msg.width)
        step = int(msg.step)
        if height <= 0 or width <= 0 or step <= 0:
            raise ValueError(
                f'Aurora图像尺寸无效: {width}x{height}, step={step}')

        raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)

        def packed(channels):
            required = height * step
            if raw.size < required or step < width * channels:
                raise ValueError(
                    f'Aurora {encoding} 数据长度不匹配: {raw.size}, step={step}')
            rows = raw[:required].reshape(height, step)
            return rows[:, :width * channels].reshape(height, width, channels)

        if encoding == 'bgr8':
            return packed(3).copy()
        if encoding == 'rgb8':
            return cv2.cvtColor(packed(3), cv2.COLOR_RGB2BGR)
        if encoding == 'bgra8':
            return cv2.cvtColor(packed(4), cv2.COLOR_BGRA2BGR)
        if encoding == 'rgba8':
            return cv2.cvtColor(packed(4), cv2.COLOR_RGBA2BGR)
        if encoding in ('mono8', '8uc1'):
            mono = packed(1).reshape(height, width)
            return cv2.cvtColor(mono, cv2.COLOR_GRAY2BGR)
        if encoding in ('yuyv', 'yuy2', 'yuv422_yuy2'):
            return cv2.cvtColor(packed(2), cv2.COLOR_YUV2BGR_YUY2)
        if encoding in ('uyvy', 'yuv422'):
            return cv2.cvtColor(packed(2), cv2.COLOR_YUV2BGR_UYVY)
        if encoding == 'nv12':
            required_rows = height * 3 // 2
            required = required_rows * step
            if raw.size < required or step < width:
                raise ValueError(
                    f'Aurora NV12数据长度不匹配: {raw.size}, step={step}')
            nv12 = raw[:required].reshape(required_rows, step)[:, :width]
            return cv2.cvtColor(nv12, cv2.COLOR_YUV2BGR_NV12)
        raise ValueError(f'暂不支持Aurora图像编码: {msg.encoding!r}')

    def _prepare_jpeg(self, camera, frame):
        if camera == 'front':
            image = self._decode_compressed(frame)
        else:
            image = self._decode_raw(frame)
            image = cv2.rotate(image, cv2.ROTATE_180)

        interpolation = (
            cv2.INTER_AREA
            if image.shape[1] > self.output_width or image.shape[0] > self.output_height
            else cv2.INTER_LINEAR
        )
        image = cv2.resize(
            image, (self.output_width, self.output_height), interpolation=interpolation)
        ok, encoded = cv2.imencode(
            '.jpg', image, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        if not ok:
            raise ValueError('OpenCV无法编码待上传JPEG')

        if self.debug_image_path:
            directory = os.path.dirname(self.debug_image_path)
            if directory:
                os.makedirs(directory, exist_ok=True)
            cv2.imwrite(self.debug_image_path, image)
        return encoded.tobytes()

    def _request(self, context, parts, timeout_ms):
        socket = context.socket(zmq.REQ)
        socket.setsockopt(zmq.LINGER, 0)
        socket.connect(f'tcp://{self.server_ip}:{self.server_port}')
        poller = zmq.Poller()
        poller.register(socket, zmq.POLLIN)
        try:
            socket.send_multipart(parts)
            if not poller.poll(timeout_ms):
                return None
            return socket.recv_multipart()
        finally:
            poller.unregister(socket)
            socket.close()

    @staticmethod
    def _parse_reply(reply, request_id):
        if reply is None:
            return None, ''
        if len(reply) != 3:
            raise RuntimeError(f'PC回包段数错误: {len(reply)}，应为3')
        reply_id, status, text = reply
        if reply_id != request_id:
            raise RuntimeError('PC回包ID与当前图片不匹配')
        return status, text.decode('utf-8', 'replace')

    def _exchange_with_pc(self, image_bytes):
        context = zmq.Context()
        request_id = uuid.uuid4().hex.encode('ascii')
        try:
            uploaded = False
            for attempt in range(1, self.request_retries + 1):
                if self._stop_event.is_set():
                    raise RuntimeError('节点正在退出')
                reply = self._request(
                    context,
                    [request_id, b'UPLOAD', image_bytes],
                    self.upload_timeout_ms,
                )
                status, text = self._parse_reply(reply, request_id)
                if status is None:
                    self._queue_status(
                        f'PC未确认收到图片，正在重试（{attempt}/{self.request_retries}）')
                    self._stop_event.wait(0.1)
                    continue
                if status == b'OK':
                    return text
                if status == b'ACK':
                    uploaded = True
                    break
                raise RuntimeError(f'PC拒绝图片或推理失败: {text}')

            if not uploaded:
                raise RuntimeError(f'图片上传失败，已重试{self.request_retries}次')

            self._queue_status('PC已收到图片，正在等待推理结果')
            if self._stop_event.wait(self.result_poll_initial_delay):
                raise RuntimeError('节点正在退出')

            interval = self.result_poll_interval
            for _ in range(self.result_poll_retries):
                if self._stop_event.is_set():
                    raise RuntimeError('节点正在退出')
                reply = self._request(
                    context, [request_id, b'GET'], self.poll_timeout_ms)
                status, text = self._parse_reply(reply, request_id)
                if status == b'OK':
                    return text
                if status not in (None, b'PENDING'):
                    raise RuntimeError(f'PC推理失败: {text}')
                if self._stop_event.wait(interval):
                    raise RuntimeError('节点正在退出')
                interval = min(interval * 2.0, self.result_poll_max_interval)

            raise RuntimeError('等待PC推理结果超时')
        finally:
            context.term()

    def _worker_loop(self):
        while not self._stop_event.is_set():
            try:
                camera = self._jobs.get(timeout=0.2)
            except queue.Empty:
                continue
            if camera is None:
                break

            try:
                frame = self._wait_for_current_frame(camera)
                if frame is None:
                    camera_name = '前置' if camera == 'front' else 'Aurora'
                    raise RuntimeError(
                        f'{self.camera_wait_timeout:.1f}秒内没有收到{camera_name}相机新画面')
                image_bytes = self._prepare_jpeg(camera, frame)
                self._queue_status(
                    f'图片已准备，正在上传到PC（{len(image_bytes) / 1024.0:.1f} KiB）')
                result = self._exchange_with_pc(image_bytes).strip()
                if not result:
                    raise RuntimeError('PC返回了空的推理结果')
                self._events.put(('result', result))
            except Exception as error:  # Keep the worker alive for a later race run.
                self._events.put(('error', str(error)))
            finally:
                with self._frame_lock:
                    self._request_active = False
                self._jobs.task_done()

    def _queue_status(self, text):
        self._events.put(('status', text))

    def _publish_worker_events(self):
        while True:
            try:
                event, text = self._events.get_nowait()
            except queue.Empty:
                return

            message = String()
            message.data = text
            if event == 'result':
                self.result_pub.publish(message)
                status = String()
                status.data = 'PC推理完成，结果已发送到屏幕'
                self.status_pub.publish(status)
                self.get_logger().warning(f'收到PC推理结果: {text}')
            else:
                self.status_pub.publish(message)
                if event == 'error':
                    self.get_logger().error(f'环形区拍照任务失败: {text}')
                else:
                    self.get_logger().info(text)

    def destroy_node(self):
        self._stop_event.set()
        try:
            self._jobs.put_nowait(None)
        except queue.Full:
            pass
        self._worker.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RingPhotoClient()
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
