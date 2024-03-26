import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import cv2 as cv
from cv_bridge import CvBridge
from setuptools import setup


class ImageFilterNode(Node):
    def __init__(self):
        super().__init__('image_filter_node')
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )
        self.publisher = self.create_publisher(
            Image,
            '/image_filtered',
            10
        )

    def image_callback(self, msg):
        # Perform image filtering to find the center
        # Add your filtering logic here
        self.get_logger().info('Image received')
        img = CvBridge().imgmsg_to_cv2(msg, 'bgr8')
        h, w, c = img.shape
        lower_res = img
        # lower_res = cv.pyrDown(img, dstsize=(w // 2, h // 2))
        cropimg = lower_res[0:int(lower_res.shape[0]*0.8)]
        blurimg = cv.GaussianBlur(cropimg, (5, 5), 0)
        lab = cv.cvtColor(blurimg, cv.COLOR_BGR2HSV)
        mask = cv.inRange(lab, (0, 50, 120), (255, 255, 255))
        res = cv.bitwise_and(blurimg, blurimg, mask=mask)
        self.publisher.publish(CvBridge().cv2_to_imgmsg(res, 'bgr8'))


def main(args=None):
    rclpy.init(args=args)
    node = ImageFilterNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
