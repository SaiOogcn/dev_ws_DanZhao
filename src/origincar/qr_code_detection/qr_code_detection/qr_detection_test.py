import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class QRCodeRecognizer(Node):
    def __init__(self):
        super().__init__('qr_code_recognizer')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10)

        # Load the template images
        self.clockwise_template = cv2.imread('/root/dev_ws/src/origincar/qr_code_detection/qrcode_image/clockwise_template.png', 0)
        self.counterclockwise_template = cv2.imread('/root/dev_ws/src/origincar/qr_code_detection/qrcode_image/counterclockwise_template.png', 0)

        # Precompute key regions
        self.key_regions = self.compute_key_regions(self.clockwise_template, self.counterclockwise_template)

    def compute_key_regions(self, img1, img2):
        diff = cv2.absdiff(img1, img2)
        _, thresholded_diff = cv2.threshold(diff, 50, 255, cv2.THRESH_BINARY)
        keypoints = cv2.findNonZero(thresholded_diff)
        return keypoints

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        self.process_image(gray_image)

    def process_image(self, image):
        # Focus only on key regions for faster matching
        clockwise_match = self.match_key_regions(image, self.clockwise_template)
        counterclockwise_match = self.match_key_regions(image, self.counterclockwise_template)

        if clockwise_match > counterclockwise_match:
            self.get_logger().info('Detected clockwise QR code')
        else:
            self.get_logger().info('Detected counterclockwise QR code')

    def match_key_regions(self, image, template):
        match_sum = 0
        for kp in self.key_regions:
            x, y = kp[0]
            if y + template.shape[0] <= image.shape[0] and x + template.shape[1] <= image.shape[1]:
                region = image[y:y + template.shape[0], x:x + template.shape[1]]
                match = cv2.matchTemplate(region, template, cv2.TM_CCOEFF_NORMED)
                _, max_val, _, _ = cv2.minMaxLoc(match)
                match_sum += max_val
        return match_sum

def main(args=None):
    rclpy.init(args=args)
    node = QRCodeRecognizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
