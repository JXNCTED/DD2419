import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from aruco_msgs.msg import MarkerArray
from std_msgs.msg import Int16MultiArray
from numpy import *

import math


class ArucoDetectPick(Node):
    def __init__(self):
        super().__init__('arm_detect_object_pick_up')
        self.delay_cnt_neutral = 0

        self.neutral_timer = self.create_timer(
            0.01, self.neutral_timer_callback)

        self.arm_pub_ = self.create_publisher(
            Int16MultiArray, '/multi_servo_cmd_sub', 10)

    def theta_1_value(self, degrees):
        value_per_deg = (20000-4000)/180
        return 4000+int(degrees*value_per_deg)

    def theta_2_value(self, degrees):
        value_per_deg = (3000-21000)/180
        return 21000+int(degrees*value_per_deg)

    def theta_3_value(self, degrees):
        value_per_deg = (20000-3000)/180
        return 3000+int(degrees*value_per_deg)

    def inverse_kin(self):
        # Length of links in cm
        a1 = 10.1
        a2 = 9.4
        a3 = 16.9

        # Desired Position of End effector
        px = -14
        py = 3

        phi = 90
        phi = deg2rad(phi)

        # Equations for Inverse kinematics
        wx = px - a3*cos(phi)
        wy = py - a3*sin(phi)

        delta = wx**2 + wy**2
        c2 = (delta - a1**2 - a2**2)/(2*a1*a2)
        s2 = sqrt(1-c2**2)  # elbow down
        theta_2 = arctan2(s2, c2)

        s1 = ((a1+a2*c2)*wy - a2*s2*wx)/delta
        c1 = ((a1+a2*c2)*wx + a2*s2*wy)/delta
        theta_1 = arctan2(s1, c1)
        theta_3 = phi-theta_1-theta_2

        print('theta_1: ', rad2deg(theta_1))
        print('theta_2: ', rad2deg(theta_2))
        print('theta_3: ', rad2deg(theta_3))

    def neutral_timer_callback(self):
        # Set the arm to the new neutral. "LOOKING DOWN all the time"
        self.get_logger().info(f"{self.delay_cnt_neutral}")
        self.delay_cnt_neutral += 1
        if self.delay_cnt_neutral < 500:
            arm_cmd = Int16MultiArray()
            arm_cmd.data = [12000 for i in range(12)]

            print(self.theta_1_value(-97))
            print(self.theta_2_value(97))
            print(self.theta_3_value(106))

            # ID on this servo is 5 THETA_1 4000IS ABOUT 0 DEGREES.
            # ADD 90 degrees here!!! Important since our robot is angled 90 degrees more than the youtube/github solution.
            arm_cmd.data[4] = self.theta_1_value(-67+90)
            """For the servo at the bottom. The least theta_1 is with a value of 4000. 90 is 12000 and 180 is 20000"""
            """Middle servo, theta_2, the bottom 0deg is with 21000 and the maximum theta=180 is with 3000 """
            """Top servo. theta_3. 0deg= 3000, 180 deg= 20000"""
            # ID on servo is 4 THETA_2
            arm_cmd.data[3] = self.theta_2_value(53)
            # Two is actually three (going by Id on the servos)
            arm_cmd.data[2] = self.theta_3_value(114)  # THETA_3

            # set the speed of all the controls
            for i in range(6):
                arm_cmd.data[i+6] = 1000
            arm_cmd.data[0] = 0
            self.arm_pub_.publish(arm_cmd)
        else:
            self.has_set_neutral = True
            self.neutral_timer.cancel()

    # Everything it can see it can reach.
    # Use the /image_raw


def main():
    rclpy.init()
    node = ArucoDetectPick()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
