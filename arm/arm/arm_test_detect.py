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
        self.publisher2 = self.create_publisher(
            Image,
            '/image_filtered2',
            10
        )

    def image_callback(self, msg):
        # Perform image filtering to find the center
        # Add your filtering logic here
        self.get_logger().info('Image received')
        img = CvBridge().imgmsg_to_cv2(msg, 'bgr8')
        h, w, c = img.shape
        lower_res = img.copy()
        # lower_res = cv.pyrDown(img, dstsize=(w // 2, h // 2))
        cropimg = lower_res[0:int(lower_res.shape[0]*0.8)]
        blurimg = cv.GaussianBlur(cropimg, (5, 5), 0)
        lab = cv.cvtColor(blurimg, cv.COLOR_BGR2HSV)
        mask = cv.inRange(lab, (0, 50, 120), (255, 255, 255))
        res = cv.bitwise_and(blurimg, blurimg, mask=mask)
        self.publisher.publish(CvBridge().cv2_to_imgmsg(res, 'bgr8'))
        edge = cv.Canny(img, 150, 250)
        bluredge = cv.GaussianBlur(edge, (15, 15), 0)
        contours, _ = cv.findContours(
            edge, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        moments = [cv.moments(cnt) for cnt in contours]
        centroids = [(int(M['m10'] / (M['m00']+1e-5)), int(M['m01'] / (M['m00']+1e-5)))
                     for M in moments]
        img2 = img.copy()
        for c in centroids:
            cv.circle(img2, c, 5, (0, 0, 255), -1)
        self.publisher2.publish(CvBridge().cv2_to_imgmsg(img2, 'bgr8'))


def main(args=None):
    rclpy.init(args=args)
    node = ImageFilterNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
