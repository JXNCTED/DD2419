import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Bool, Float32, String
import numpy as np


class ArmDetect(Node):
    def __init__(self):
        super().__init__('arm_detect')

        # Create the cv_bridge object
        self.bridge = CvBridge()

        # subscribe to the raw image and then publish a filtered
        self.image_sub = self.create_subscription(
            Image, "/image_raw", self.filter_image_callback, 10)

        self.image_pub = self.create_publisher(Image, "/image_filtered", 10)
        # Wait does this even make sense since we created a node to handle these kind of things?
        self.control_arm_pub = self.create_publisher(String, '/arm_conf', 10)

        # to start the detection
        # idea is this is published when the distance is low enough (from the realsense)
        # ros2 topic pub -1 /dist_bool std_msgs/msg/Bool "{data: true}"
        self.dist_sub = self.create_subscription(
            Bool, "/dist_bool", self.dist_callback, 10)
        self.can_detect = False
        # send to know what twist message
        self.theta_pub = self.create_publisher(Float32, "/arm_theta", 10)
        self.pick_up = False

    def dist_callback(self, msg):
        self.can_detect = msg

    def filter_image_callback(self, msg: Image) -> Image:
        """
        Filters the inputted image to only display certain colours.
        """

        if self.can_detect == False:
            print("no")
            return

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

        # Find contours in the binary mask
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Calculate moments for each contour
        centroids = []
        for contour in contours:
            M = cv2.moments(contour)
            # Changed here from being NOT EQUAL TO ZERO to more than 100 to remove most of the effect of noise to the middlepoint.
            if M["m00"] > 100:
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
            radius = 3
            color = (0, 0, 255)  # red color
            cv2.circle(output, middle_coordinates, radius,
                       color, -1)  # -1 fills the circle

            # Check the amount of points?

            # Check if the middlepoint is in the middle of the image. (310-245,400-440)

            # If so, send information that the pickup phase shall begin

            # range: (310-325, 400-440)

            # Simulate current middle-point coordinates and desired area center
            desired_area_center = (320, 240)

            # Calculate the error, use this to get theta
            # set minus so that it follows normal coords
            error_x = float(desired_area_center[0] - middle_coordinates[0])
            error_y = float(desired_area_center[1] - middle_coordinates[1])
            # theta as in normal x,y coords
            theta = np.arctan2(error_y, error_x)
            self.get_logger().info(f"{theta / np.pi}")
            theta_var = theta/np.pi
            # convert to float and pub
            # Publishing the angle for controlling the wheels.
            angle = Float32()
            angle.data = theta
            self.theta_pub.publish(angle)
            if (not self.pick_up):
                if (320-32 < middle_coordinates[0] < 320+32 and 240-24 < middle_coordinates[1] < 240+24):
                    # The cube is now in a position where it hopefully can get picked up! ( ~10% margin)

                    self.get_logger().info("PICK UP! !! ")
                    self.pick_up = True
                    msg = String()
                    msg.data = "close"
                    self.control_arm_pub.publish(msg)
                    # Here the robot should then check whether or not it has the cube in its grip
                    # Which is easy, but everything else is hard.
                    # And then go into neutral mode. Or some kind of structured manouver.

                else:
                    """My though is that if the box is in the bottom half we just reverse the robot until the box
                    is in the top half and then we turn and navigate to it. I think that might be the easiest solution to it. 
                    As for calculating the angle. I am still wondering about that. Maybe it is like the wheel of life.🛞🛞🛞 """
                    """How do we transform the angle from here to the angle that we can publish to the wheels..."""
                    if (0 < theta_var < 0.45):
                        self.get_logger().info("Turn left")
                    elif (0.45 < theta_var < 0.55):
                        # In the pickup area. Publish to pick up.
                        self.get_logger().info("Go straight!  ")
                        # Set the working theta to 0, linear.x = 0.4
                    elif (-0.55 < theta_var < -0.45):
                        # In the pickup area. Publish to pick up.
                        self.get_logger().info("Just reverse")  # Theta = 0, linear.x = -0.4
                    elif (0.55 < theta_var < 1):
                        self.get_logger().info("Turn right")  # Set angle depending on
                    elif (-0.55 < theta_var < 0):
                        # Theta = 0, linear.x = -0.4
                        self.get_logger().info("BACK UP object to the left back")
                    elif (-1 < theta_var < -0.45):
                        # Theta = 0, linear.x = -0.4
                        self.get_logger().info("BACK UP object to the right back")

        # Use cv_bridge() to convert the ROS image to OpenCV format
        try:
            # Publish to arm_conf.
            filtered_image = self.bridge.cv2_to_imgmsg(output, "bgr8")
            self.image_pub.publish(filtered_image)
        except CvBridgeError as e:
            print(e)


def main():
    rclpy.init()
    node = ArmDetect()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
