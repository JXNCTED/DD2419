import numpy as np
import py_trees as pt
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridgeError
import cv2
from std_msgs.msg import Float32MultiArray

# https://medium.com/@nullbyte.in/behavior-trees-for-ros2-part-1-unlocking-advanced-robotic-decision-making-and-control-7856582fb812
# Need to create cpp file of behavior tree


class Counter(pt.behaviour.Behaviour):
    def __init__(self, count, name):
        super().__init__(name)
        self.count = count
        self.current_count = 0

    def reset_count(self):
        self.current_count = 0

    def update(self):
        print(
            f"{self.name} (Step {self.current_count}/{self.count})")

        if self.current_count < self.count:
            self.current_count += 1
            return pt.common.Status.RUNNING
        else:
            self.reset_count()  # Reset the counter when the count is completed
            return pt.common.Status.SUCCESS


class SingleCounter(pt.behaviour.Behaviour):
    def __init__(self):
        super().__init__("single_counter")
        self.current_count = 0

    def reset_count(self):
        self.current_count = 0

    def update(self):
        #
        if self.current_count < 1:
            self.current_count += 1
            print("Counting...")
            return pt.common.Status.RUNNING
        else:
            self.reset_count()  # Reset the counter when the count is completed
            return pt.common.Status.SUCCESS


class Detect(pt.behaviour.Behaviour, Node):
    def __init__(self):
        pt.behaviour.Behaviour.__init__(self, "Detect")
        Node.__init__(self, "Detect")
        self.arm_pub_msg = Int16MultiArray()
        self.arm_pub_ = self.create_publisher(
            Int16MultiArray, '/multi_servo_cmd_sub', 10)

        self.detect_angles = [12000 for i in range(12)]
        for i in range(6):
            self.detect_angles[i+6] = 1000
        self.detect_angles[2] = 2000
        self.detect_angles[3] = 18000
        self.detect_angles[4] = 10000
        self.detect_angles[0] = 5000
        self.arm_pub_msg.data = self.detect_angles

    def update(self):
        self.arm_pub_.publish(self.arm_pub_msg)
        print("Detect.")
        # succeed after count is done
        return pt.common.Status.SUCCESS


class OpenGripper(pt.behaviour.Behaviour, Node):
    def __init__(self):
        pt.behaviour.Behaviour.__init__(self, "OpenGripper")
        Node.__init__(self, "OpenGripper")
        self.arm_pub_msg = Int16MultiArray()
        self.arm_pub_ = self.create_publisher(
            Int16MultiArray, '/multi_servo_cmd_sub', 10)

        self.detect_angles = [12000 for i in range(12)]
        for i in range(6):
            self.detect_angles[i+6] = 1000
        self.detect_angles[0] = 5000
        self.detect_angles[2] = 2000
        self.detect_angles[3] = 18000
        self.detect_angles[4] = 10000
        self.arm_pub_msg.data = self.detect_angles

    def update(self):
        self.arm_pub_.publish(self.arm_pub_msg)
        print("Open gripper.")
        # succeed after count is done
        return pt.common.Status.SUCCESS


class Pickup(pt.behaviour.Behaviour, Node):
    def __init__(self):
        pt.behaviour.Behaviour.__init__(self, "Pickup")
        Node.__init__(self, "Pickup")
        self.arm_pub_msg = Int16MultiArray()
        self.arm_pub_ = self.create_publisher(
            Int16MultiArray, '/multi_servo_cmd_sub', 10)

        # Pickup phase
        self.pickup_angles = [12000 for i in range(12)]
        # set the speed of all the controls
        for i in range(6):
            self.pickup_angles[i+6] = 1000
        # set conf
        self.pickup_angles[2] = 4000
        self.pickup_angles[3] = 15500
        self.pickup_angles[4] = 5300
        self.pickup_angles[0] = 5000
        self.arm_pub_msg.data = self.pickup_angles

    def update(self):
        self.arm_pub_.publish(self.arm_pub_msg)
        print("Pickup.")
        # succeed after count is done
        return pt.common.Status.SUCCESS


class CloseGripper(pt.behaviour.Behaviour, Node):
    def __init__(self):
        pt.behaviour.Behaviour.__init__(self, "CloseGripper")
        Node.__init__(self, "CloseGripper")
        self.arm_pub_msg = Int16MultiArray()
        self.arm_pub_ = self.create_publisher(
            Int16MultiArray, '/multi_servo_cmd_sub', 10)

        # Pickup phase
        self.pickup_angles = [12000 for i in range(12)]
        # set the speed of all the controls
        for i in range(6):
            self.pickup_angles[i+6] = 1000
        # set conf
        self.pickup_angles[2] = 4000
        self.pickup_angles[3] = 15500
        self.pickup_angles[4] = 5300
        self.pickup_angles[0] = 12000
        self.arm_pub_msg.data = self.pickup_angles

    def update(self):
        self.arm_pub_.publish(self.arm_pub_msg)
        print("Close gripper.")
        # succeed after count is done
        return pt.common.Status.SUCCESS


class MoveToNeutral(pt.behaviour.Behaviour, Node):
    def __init__(self):
        pt.behaviour.Behaviour.__init__(self, "MoveToNeutral")
        Node.__init__(self, 'MoveToNeutral')
        self.arm_pub_msg = Int16MultiArray()
        self.arm_pub_ = self.create_publisher(
            Int16MultiArray, '/multi_servo_cmd_sub', 10)

        self.neutral_angles = [12000 for i in range(12)]
        for i in range(6):
            self.neutral_angles[i+6] = 1000

        self.arm_pub_msg.data = self.neutral_angles

    def update(self):
        self.arm_pub_.publish(self.arm_pub_msg)
        print("Moving to neutral.")
        # succeed after count is done
        return pt.common.Status.SUCCESS

# Am I overthinking this?


class ArmCameraSubscriber(pt.behaviour.Behaviour, Node):
    def __init__(self):
        pt.behaviour.Behaviour.__init__(self, "ArmCameraSubscriber")
        Node.__init__(self, 'ArmCameraSubscriber')

        self.image_sub = self.create_subscription(
            Image, "/image_raw", self.filter_image_callback, 10)

    def filter_image_callback(self, msg: Image) -> Image:
        """
        Filters the inputted image to only display certain colours.
        """
        if self.can_detect == False:
            self.get_logger().info("Not detecting")
            return
        else:
            self.get_logger().info("Detecting")

        # Use cv_bridge() to convert the ROS image to OpenCV format
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Set minimum and max HSV values to display
        # lower = np.array([65, 125, 94])
        # upper = np.array([80, 255, 184])
        lower = np.array([60, 110, 80])
        upper = np.array([85, 255, 200])

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

        theta_linear = Float32MultiArray()
        theta_linear.data = [0.0, 0.0]  # linear [0], theta [1]
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

            # define theta linear message to publish, default to 0
            theta_linear = Float32MultiArray()
            theta_linear.data = [0.0, 0.0]  # linear [0], theta [1]
            if (not self.pick_up):
                if (320-32 < middle_coordinates[0] < 320+32 and 240-60 < middle_coordinates[1] < 240+30):

                    # The cube is now in a position where it hopefully can get picked up! ( ~10% margin)
                    self.get_logger().info("PICK UP! !! ")
                    self.pick_up = True
                    # tell arm to shtap
                    self.timer.reset()

                    # tell wheels to sthap
                    theta_linear.data[0] = 0.0  # this is linear
                    theta_linear.data[1] = 0.0  # this is theta

                    # Here the robot should then check whether or not it has the cube in its grip
                    # Which is easy, but everything else is hard.
                    # And then go into neutral mode. Or some kind of structured manouver.
                    self.can_detect = False

                else:
                    if (0 < theta_var < 0.4):
                        self.get_logger().info("Turn left")
                        theta_linear.data[0] = 0.1  # this is linear
                        # this is theta
                        theta_linear.data[1] = -(0.5 - theta_var)
                    elif (0.6 < theta_var < 1):
                        self.get_logger().info("Turn right")    # Set angle depending on
                        theta_linear.data[0] = 0.1  # this is linear
                        theta_linear.data[1] = (
                            theta_var - 0.5)  # this is theta
                    elif (0.4 < theta_var < 0.6):
                        # In the pickup area. Publish to pick up.
                        self.get_logger().info("Go straight!  ")
                        theta_linear.data[0] = 0.1  # this is linear
                        theta_linear.data[1] = 0.0  # this is theta
                    elif (-0.6 < theta_var < -0.4):
                        self.get_logger().info("Just reverse")  # Theta = 0, linear.x = -0.4
                        theta_linear.data[0] = -0.1  # this is linear
                        theta_linear.data[1] = 0  # this is theta
                    elif (-0.4 < theta_var < 0):
                        self.get_logger().info("BACK UP object to the left back")
                        theta_linear.data[0] = -0.1  # this is linear
                        theta_linear.data[1] = 0  # this is theta
                    elif (-1 < theta_var < -0.6):
                        self.get_logger().info("BACK UP object to the right back")
                        theta_linear.data[0] = -0.1  # this is linear
                        theta_linear.data[1] = 0  # this is theta
        self.get_logger().info(f"twist pub: {theta_linear.data}")
        self.theta_linear_pub.publish(theta_linear)

        # Use cv_bridge() to convert the ROS image to OpenCV format
        try:
            # Publish to arm_conf.
            filtered_image = self.bridge.cv2_to_imgmsg(output, "bgr8")
            self.image_pub.publish(filtered_image)
        except CvBridgeError as e:
            print(e)

    def update(self):
        print("ArmCameraSubscriber.")
        return pt.common.Status.SUCCESS


def main():
    rclpy.init()
    node = rclpy.create_node("behaviour_tree_example")

    sequence_list = [
        SingleCounter(), Detect(),
        SingleCounter(), OpenGripper(),
        SingleCounter(), Pickup(),
        SingleCounter(), CloseGripper(),
        SingleCounter(), MoveToNeutral(),
    ]

    root = pt.composites.Sequence("Root", True)
    root.add_children(sequence_list)
    tree = pt.trees.BehaviourTree(root)

    final_status, details = tree.tick_tock(
        period_ms=3000,
        number_of_iterations=pt.trees.CONTINUOUS_TICK_TOCK,
    )

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
