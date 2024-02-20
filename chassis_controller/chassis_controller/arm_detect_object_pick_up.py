import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from aruco_msgs.msg import MarkerArray
from std_msgs.msg import Int16MultiArray
import numpy as np

import math


class ArucoDetectPick(Node):
    def __init__(self):
        super().__init__('arm_detect_object_pick_up')
        self.delay_cnt_neutral = 0

        self.neutral_timer = self.create_timer(
            0.01, self.neutral_timer_callback)

        self.arm_pub_ = self.create_publisher(
            Int16MultiArray, '/multi_servo_cmd_sub', 10)

        self.neutral_angles = [12000 for i in range(12)]
        # set the speed of all the controls
        for i in range(6):
            self.neutral_angles[i+6] = 1000

        # HEAD DOWN ASS UP, THAT'S THE WAY WE LIKE TO ...
        self.detect_angles = [12000 for i in range(12)]
        # set the speed of all the controls
        for i in range(6):
            self.detect_angles[i+6] = 1000
        self.detect_angles[2] = 2000
        self.detect_angles[3] = 18000
        self.detect_angles[4] = 10000
        self.detect_angles[0] = 5000

        # Pick up shit...
        self.pickup_angles = [12000 for i in range(12)]
        # set the speed of all the controls
        for i in range(6):
            self.pickup_angles[i+6] = 1000
        self.pickup_angles[2] = 4000
        self.pickup_angles[3] = 15400
        self.pickup_angles[4] = 6160

        self.open_gripper = 5000
        self.close_gripper = 12000

    def theta_1_value(self, degrees):
        return np.max([4000, np.min([20000, int(12000-(degrees-90)*100)])])

    def theta_2_value(self, degrees):
        return np.max([4000, np.min([20000, int(12000+(degrees)*100)])])

    def theta_3_value(self, degrees):
        return np.max([1000, np.min([23000, int(12000-(degrees)*100)])])

    def inverse_kin(self):
        # Length of links in cm
        a1 = 10.1
        a2 = 9.4
        a3 = 16.9

        px = 10  # With this phi conf, it can only handle 18-15cm
        px = -px
        py = -5
        # 18, -14 for pickup

        phi = 4.712  # there are only some valid phi values, check the desmos

        # Equations for Inverse kinematics
        wx = px - a3*np.cos(phi)
        wy = py - a3*np.sin(phi)

        delta = wx**2 + wy**2
        c2 = (delta - a1**2 - a2**2)/(2*a1*a2)

        s2 = np.sqrt(1-c2**2)  # elbow down
        theta_2 = np.arctan2(s2, c2)

        s1 = ((a1+a2*c2)*wy - a2*s2*wx)/delta
        c1 = ((a1+a2*c2)*wx + a2*s2*wy)/delta
        theta_1 = np.arctan2(s1, c1)
        theta_3 = phi-theta_1-theta_2

        print('theta_1: ', np.rad2deg(theta_1))
        print('theta_2: ', np.rad2deg(theta_2))
        print('theta_3: ', np.rad2deg(theta_3))

        return np.rad2deg(theta_1), np.rad2deg(theta_2), np.rad2deg(theta_3)

    def neutral_timer_callback(self):
        # Set the arm to the new neutral. "LOOKING DOWN all the time"
        self.get_logger().info(f"{self.delay_cnt_neutral}")
        self.delay_cnt_neutral += 1
        if self.delay_cnt_neutral < 750:
            arm_cmd = Int16MultiArray()
            arm_cmd.data = [12000 for i in range(12)]

            theta_1, theta_2, theta_3 = self.inverse_kin()

            theta_1_16bit = self.theta_1_value(theta_1)
            theta_2_16bit = self.theta_2_value(theta_2)
            theta_3_16bit = self.theta_3_value(theta_3)

            print(
                f"servo1: {theta_1_16bit}, servo2: {theta_2_16bit}, servo3: {theta_3_16bit}")

            # ID on this servo is 5 THETA_1 4000IS ABOUT 0 DEGREES.
            # ADD 90 degrees here!!! Important since our robot is angleddetect_angles 90 degrees more than the youtube/github solution.
            #
            # arm_cmd.data[4] = self.theta_1_value(-67+90)
            bottom_servo = arm_cmd.data[4]
            arm_cmd.data[4] = theta_1_16bit

            """For the servo at the bottom. The least theta_1 is with a value of 4000. 90 is 12000 and 180 is 20000"""
            """Middle servo, theta_2, the bottom 0deg is with 21000 and the maximum theta=180 is with 3000 """
            """Top servo. theta_3. 0deg= 3000, 180 deg= 20000"""
            # ID on servo is 4 THETA_2
            arm_cmd.data[3] = theta_2_16bit

            # Two is actually three (going by Id on the servos)
            arm_cmd.data[2] = theta_3_16bit  # THETA_3
            # set the speed of all the controls
            for i in range(6):
                arm_cmd.data[i+6] = 1000

            arm_cmd.data[0] = self.close_gripper
            # arm_cmd.data[4] = 12000
            # arm_cmd.data[3] = 12000
            # arm_cmd.data[2] = 12000

            # arm_cmd.data[5] = 15000            #ROTATION OF THE BASE

            # PREDEFINED POSITION FOR THE ARM
            # arm_cmd.data = self.neutral_angles  # STRAIGHT UP
            # arm_cmd.data = self.detect_angles  # DETECTING MODE, SEARCHING
            # arm_cmd.data = self.pickup_angles  # PICKING UP
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
