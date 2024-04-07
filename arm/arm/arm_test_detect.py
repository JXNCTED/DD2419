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

        # hardcoded camera parameters
        self.K = np.array([[513.34301, 0., 307.89617],
                           [0., 513.84807, 244.62007],
                           [0., 0., 1.]])

        self.coeffs = np.array(
            [-0.474424, 0.207336, -0.002361, 0.000427, 0.000000])

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
        # self.get_logger().info('Image received')

        img = CvBridge().imgmsg_to_cv2(msg, 'bgr8')
        img = cv.undistort(img, self.K, self.coeffs)

        img = cv.bilateralFilter(img, 20, 100, 100)

        copy = img.copy()

        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        edges = cv.Canny(gray, 50, 150, apertureSize=3)

        contours, _ = cv.findContours(
            edges, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)
        # combine all contours into one
        if len(contours) > 0:
            contours = np.concatenate(contours)
            cv.drawContours(copy, contours, -1, (0, 255, 0), 3)
            rect = cv.minAreaRect(contours)
            box = cv.boxPoints(rect)
            box = np.int0(box)
            centroid = np.mean(box, axis=0)
            cv.drawContours(copy, [box], 0, (0, 0, 255), 2)
            cv.circle(copy, (int(centroid[0]), int(
                centroid[1])), 10, (0, 0, 255), -1)
            line = cv.fitLine(contours, cv.DIST_L2, 0, 0.01, 0.01)
            lefty = int((-line[2] * line[1] / line[0]) + line[3])
            righty = int(((gray.shape[1] - line[2])
                         * line[1] / line[0]) + line[3])
            cv.line(copy, (0, lefty),
                    (gray.shape[1] - 1, righty), (255, 0, 0), 2)

        img_msg = CvBridge().cv2_to_imgmsg(copy, 'bgr8')
        self.publisher.publish(img_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImageFilterNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
