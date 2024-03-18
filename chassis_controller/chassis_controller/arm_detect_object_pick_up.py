import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from aruco_msgs.msg import MarkerArray
from std_msgs.msg import Int16MultiArray


import math


class ArucoDetectPick(Node):
    def __init__(self):
        super().__init__('arm_detect_object_pick_up')
        self.delay_cnt_neutral = 0

        self.neutral_timer = self.create_timer(
            0.01, self.neutral_timer_callback)

        self.arm_pub_ = self.create_publisher(
            Int16MultiArray, '/multi_servo_cmd_sub', 10)

    def neutral_timer_callback(self):
        # Set the arm to the new neutral. "LOOKING DOWN all the time"
        self.get_logger().info(f"{self.delay_cnt_neutral}")
        self.delay_cnt_neutral += 1
        if self.delay_cnt_neutral < 500:
            arm_cmd = Int16MultiArray()
            arm_cmd.data = [12000 for i in range(12)]
            arm_cmd.data[3] = 17000
            arm_cmd.data[4] = 8000
            arm_cmd.data[2] = 3000
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
