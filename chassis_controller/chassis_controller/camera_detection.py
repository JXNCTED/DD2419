import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


class CameraDetection(Node):
    def __init__(self):
        super().__init__('camera_detect_object')

        # Create the cv_bridge object
        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(
            Image, "/image_raw", self.filter_image_callback, 10)

        self.image_pub = self.create_publisher(Image, "/image_filtered", 10)

    def filter_image_callback(self, msg: Image) -> Image:
        """
        Filters the inputted image to only display certain colours.
        """

        # Use cv_bridge() to convert the ROS image to OpenCV format
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Set minimum and max HSV values to display
        lower = np.array([65, 125, 94])
        upper = np.array([80, 255, 184])

        # Create HSV Image and threshold into a range.
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower, upper)
        output = cv2.bitwise_and(frame, frame, mask=mask)

        # Use cv_bridge() to convert the ROS image to OpenCV format
        try:
            filtered_image = self.bridge.cv2_to_imgmsg(output, "bgr8")
            self.image_pub.publish(filtered_image)
        except CvBridgeError as e:
            print(e)


def main():
    rclpy.init()
    node = CameraDetection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
