#!/usr/bin/env python
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16MultiArray


class JoyController(Node):
    def __init__(self):
        super().__init__('joy_controller')
        # motor publisher
        self.twist_pub_ = self.create_publisher(
            Twist, '/motor_controller/twist', 10)
        # servo publisher
        self.arm_pub_ = self.create_publisher(
            Int16MultiArray, '/multi_servo_cmd_sub', 10)
        # input subscriber
        self.joy_sub_ = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10)
        # motor settings
        self.MAX_V = 0.7
        self.MAX_W = 2.0
        self.prev_twist_pub_msg = []
        # arm settings
        self.prev_arm_pub_msg = []

    def joy_callback(self, msg: Joy):
        # ----------------------------------MOTORS--------------------------
        # set speed
        v = self.MAX_V * msg.axes[1]  # right vertical
        w = self.MAX_W * msg.axes[3]  # right horizontal

        # create and publish twist
        twist_pub_msg = Twist()
        twist_pub_msg.angular.z = w
        twist_pub_msg.linear.x = v
        # publish to topic only when we change some values
        # then reset to new
        if twist_pub_msg != self.prev_twist_pub_msg:
            self.twist_pub_.publish(twist_pub_msg)
            self.prev_twist_pub_msg = twist_pub_msg
            print("Changing movement!")

        # ------------------------------------ARM--------------------------------

        arm_pub_msg = Int16MultiArray()

        # default to middle position
        arm_pub_msg.data = [12000 for i in range(12)]
        # set the speed of all the controls
        for i in range(6):
            arm_pub_msg.data[i+6] = 1000

        # open-close gripper
        # Right bumper button is pressed on the controller
        right_bumper_pressed = int(msg.buttons[5])
        if (right_bumper_pressed == 1):
            arm_pub_msg.data[0] = 12000  # grab
            print("Gripping!")
        else:
            arm_pub_msg.data[0] = 2000  # release

        # move joints with left bumper
        left_bumper_pressed = int(msg.buttons[4])
        # hold left bumper down, move arm down
        if left_bumper_pressed == 1:
            arm_pub_msg.data[3] = 18000
            arm_pub_msg.data[2] = 8000
            arm_pub_msg.data[4] = 6500
            print("Moving arm!")

        # publish to topic only when we change some values
        # then reset to new
        if arm_pub_msg.data != self.prev_arm_pub_msg:
            self.arm_pub_.publish(arm_pub_msg)
            self.prev_arm_pub_msg = arm_pub_msg.data

        # What are the different msg.buttons[]. Msg.buttons only has 11 elements but it should have 20


def main():
    rclpy.init()
    node = JoyController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:        # if twist_pub_msg != self.prev_twist_pub_msg:
        #     self.twist_pub_.publish(twist_pub_msg)
        #     self.prev_twist_pub_msg = twist_pub_msg
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()


"""
0   A (CROSS)
1 	B (CIRCLE)
2 	X (SQUARE)
3 	Y (TRIANGLE)
4 	BACK (SELECT)
5 	GUIDE (Middle/Manufacturer Button)
6 	START
7 	LEFTSTICK  -- Which does not seem to be reachable
8 	RIGHTSTICK
9 	LEFTSHOULDER
10 	RIGHTSHOULDER
11 	DPAD_UP
12 	DPAD_DOWN
13 	DPAD_LEFT
14 	DPAD_RIGHT
15 	MISC1 (Depends on the controller manufacturer, but is usually at a similar location on the controller as back/start)
16 	PADDLE1 (Upper left, facing the back of the controller if present)
17 	PADDLE2 (Upper right, facing the back of the controller if present)
18 	PADDLE3 (Lower left, facing the back of the controller if present)
19 	PADDLE4 (Lower right, facing the back of the controller if present)
20 	TOUCHPAD (If present. Button status only)"""
