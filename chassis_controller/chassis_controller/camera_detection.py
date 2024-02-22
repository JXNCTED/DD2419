import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from robp_interfaces.msg import PickAndPlace


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
        # lower = np.array([68, 67, 138])
        # upper = np.array([86, 108, 170])
        lower = np.array([65, 125, 94])
        upper = np.array([80, 255, 184])

        # Create HSV Image and threshold into a range.
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower, upper)
        output = cv2.bitwise_and(frame, frame, mask=mask)

        # Find contours in the binary mask
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # # Draw contours on the original image
        # cv2.drawContours(frame, contours, -1, (0, 255, 0), 2)

        # Calculate moments for each contour
        centroids = []
        for contour in contours:
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                centroids.append((cx, cy))

        if centroids:
            # Calculate the average centroid (middle point)
            middle_coordinates = (
                int(np.mean([cx for cx, _ in centroids])),
                int(np.mean([cy for _, cy in centroids]))
            )

            self.get_logger().info(f"Middle Coordinates: {middle_coordinates}")

            # Display a point at the middle coordinates
            radius = 3  # Adjust the radius based on your preference
            color = (0, 0, 255)  # Green color, you can adjust this as well
            cv2.circle(output, middle_coordinates, radius,
                       color, -1)  # -1 fills the circle

        # Check if the middlepoint is in the middle of the image. (310-245,400-440)
            # If so, send information that the pickup phase shall begin

            # range: (310-325, 400-440)
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
