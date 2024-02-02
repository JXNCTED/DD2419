#!/usr/bin/env python
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16MultiArray

class JoyController(Node):
    def __init__(self):
        super().__init__('joy_controller')
        self.twist_pub_ = self.create_publisher(Twist, '/motor_controller/twist', 10)
        self.arm_pub_ = self.create_publisher(Int16MultiArray, '/multi_servo_cmd_sub', 10)
        self.joy_sub_ = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.MAX_V = 0.7
        self.MAX_W = 2.0
    
    def joy_callback(self, msg: Joy):
        v = self.MAX_V * msg.axes[3] # right vertical
        w = self.MAX_W * msg.axes[2] # right horizontal
        twist_pub_msg = Twist()
        twist_pub_msg.angular.z = w
        twist_pub_msg.linear.x = v
        self.twist_pub_.publish(twist_pub_msg)

        arm_pub_msg = Int16MultiArray()
        # unused data to negative
        arm_pub_msg.data = [-1 for i in range(12)]
        B_pressed = int(msg.buttons[2]) # red B button on the joy
        if (B_pressed == 1):
            arm_pub_msg.data[0] = 14000 # grab
        else:
            arm_pub_msg.data[0] = 2000 # release

        


        for i in range(6):
            arm_pub_msg.data[i+6] = 500
        self.arm_pub_.publish(arm_pub_msg)
            




def main():
    rclpy.init()
    node = JoyController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()




if __name__ == '__main__':
    main()

