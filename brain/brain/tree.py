import numpy as np
import py_trees as pt
import py_trees_ros as ptr
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridgeError
import cv2
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import Bool, Float32, String, Float32MultiArray


# https://medium.com/@nullbyte.in/behavior-trees-for-ros2-part-1-unlocking-advanced-robotic-decision-making-and-control-7856582fb812
# Need to create cpp file of behavior tree


class Counter(pt.behaviour.Behaviour):
    def __init__(self, count):
        super().__init__("counter")
        self.count = count
        self.current_count = 0

    def reset_count(self):
        self.current_count = 0

    def update(self):
        print(
            f"(Step {self.current_count}/{self.count})")

        if self.current_count < self.count:
            self.current_count += 1
            return pt.common.Status.RUNNING
        else:
            self.reset_count()  # Reset the counter when the count is completed
            return pt.common.Status.SUCCESS


# This is just to move the robot from getting a coordinate.
# We do not need to have this node. Add it to the bringup instead.

# This node should check if the object in the arm camera is centered via a topic from arm_detect node (ran separately)
class CheckForObjects(ptr.behaviour.Behaviour, Node):
    def __init__(self):
        pt.behaviour.Behaviour.__init__(self, "CheckForObjects")
        Node.__init__(self, "CheckForObjects")
        # Have random topic here. THis topic will have true or false values for if the object is in the middle.
        self.is_object_centered_sub = self.create_subscription(
            Bool, "/is_object_centered", self.is_object_centered_callback, 10)
        # self.is_object_centered = False
        self.is_object_centered = False
        self.latest_message = None

    def update(self):
        # Manually read from the topic
        # Non-blocking read. Doesn't work...

        if self.latest_message:
            self.is_object_centered_callback(self.latest_message)

        print(self.is_object_centered)

        if self.is_object_centered:
            print("Object is centered")
            return pt.common.Status.SUCCESS
        else:
            print("Object is not centered")
            return pt.common.Status.RUNNING

    def is_object_centered_callback(self, msg):
        try:
            print(msg.data)
            self.is_object_centered = msg.data
            self.latest_message = msg
            print("Received message:", msg.data)
        except Exception as e:
            print("Error in callback:", e)


class ReconfigureArm(pt.behaviour.Behaviour, Node):
    def __init__(self, configuration, gripper):
        pt.behaviour.Behaviour.__init__(self, "Configure")
        Node.__init__(self, "Configure")
        self.arm_pub_msg = Int16MultiArray()
        self.arm_pub_ = self.create_publisher(
            Int16MultiArray, '/multi_servo_cmd_sub', 10)

        self.configuration = configuration

        # default to neutral
        self.arm_angles = [12000 for i in range(12)]
        # set the speed of all the controls
        for i in range(6):
            self.arm_angles[i+6] = 1000

        # set phase
        if self.configuration == "detect":
            self.arm_angles[2] = 2000
            self.arm_angles[3] = 18000
            self.arm_angles[4] = 10000
        elif self.configuration == "pickup":
            self.arm_angles[2] = 4000
            self.arm_angles[3] = 15500
            self.arm_angles[4] = 5300
        # set gripper state
        if gripper == "open":
            self.arm_angles[0] = 5000
        elif gripper == "close":
            self.arm_angles[0] = 12000

        self.arm_pub_msg.data = self.arm_angles

    def update(self):
        self.arm_pub_.publish(self.arm_pub_msg)
        print(self.configuration)
        # succeed after count is done
        return pt.common.Status.SUCCESS


def main():
    rclpy.init()
    node = rclpy.create_node("behaviour_tree_example")
    sequence_list = [
        Counter(1), ReconfigureArm("detect", "open"),
        Counter(1), CheckForObjects(),  # Works?!
        Counter(1), ReconfigureArm("pickup", "open"),
        Counter(1), ReconfigureArm("pickup", "close"),
        Counter(1), ReconfigureArm(None, "close"),  # None is neutral
    ]

    # Implement the same logic that we used before in this tree
    # Create a node for handling the detecting and moving?
    # Run the arm_detect_move node

    root = pt.composites.Sequence("Root", True)
    root.add_children(sequence_list)
    tree = pt.trees.BehaviourTree(root)

    print(pt.display.unicode_tree(root))

    final_status, details = tree.tick_tock(
        period_ms=3000,
        number_of_iterations=pt.trees.CONTINUOUS_TICK_TOCK,
    )

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
