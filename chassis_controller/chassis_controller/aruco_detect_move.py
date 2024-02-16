import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from aruco_msgs.msg import MarkerArray
from std_msgs.msg import Int16MultiArray


import math


class ArucoDetectMove(Node):
    def __init__(self):
        # initialize publisher
        # Initialize subscriber
        super().__init__('aruco_detect_move')

        self.target_reached = False
        self.delay_cnt = 0
        self.subscription = self.create_subscription(
            MarkerArray,
            '/marker_publisher/markers',
            self.target_theta_callback,
            10
        )
        self.publishCoordinates = self.create_publisher(
            Twist,
            '/motor_controller/twist',
            10
        )
        # self.arm_pub_ = self.create_publisher(
        #     JointState, '/servo_joint_space_target', 10)
        # timer for movements after first sequence
        self.arm_pub_ = self.create_publisher(
            Int16MultiArray, '/multi_servo_cmd_sub', 10)
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.timer.cancel()
        # first sequence, picking up object
        self.has_set_neutral = False
        self.delay_cnt_neutral = 0
        self.neutral_timer = self.create_timer(
            0.01, self.neutral_timer_callback)

    def neutral_timer_callback(self):
        self.get_logger().info(f"{self.delay_cnt_neutral}")
        self.delay_cnt_neutral += 1
        if self.delay_cnt_neutral < 500:
            arm_cmd = Int16MultiArray()
            arm_cmd.data = [12000 for i in range(12)]
            # set the speed of all the controls
            for i in range(6):
                arm_cmd.data[i+6] = 1000
            arm_cmd.data[0] = 0
            self.arm_pub_.publish(arm_cmd)
        elif self.delay_cnt_neutral < 1000:
            arm_cmd = Int16MultiArray()
            arm_cmd.data = [12000 for i in range(12)]
            # set the speed of all the controls
            for i in range(6):
                arm_cmd.data[i+6] = 1000

            arm_cmd.data[0+6] = 500
            arm_cmd.data[0] = 11000

            arm_cmd.data[3] = 18000
            arm_cmd.data[2] = 8000
            arm_cmd.data[4] = 6500

            arm_cmd.data[0] = 0
            self.arm_pub_.publish(arm_cmd)
        elif self.delay_cnt_neutral < 1500:
            arm_cmd = Int16MultiArray()
            arm_cmd.data = [12000 for i in range(12)]
            # set the speed of all the controls
            for i in range(6):
                arm_cmd.data[i+6] = 1000
            arm_cmd.data[0+6] = 500
            arm_cmd.data[0] = 11000

            arm_cmd.data[3] = 18000
            arm_cmd.data[2] = 8000
            arm_cmd.data[4] = 6500

            arm_cmd.data[0] = 11000
            self.arm_pub_.publish(arm_cmd)
        elif self.delay_cnt_neutral < 2000:
            arm_cmd = Int16MultiArray()
            arm_cmd.data = [12000 for i in range(12)]
            # set the speed of all the controls
            for i in range(6):
                arm_cmd.data[i+6] = 1000
            arm_cmd.data[0] = 11000
            self.arm_pub_.publish(arm_cmd)
        else:
            self.has_set_neutral = True
            self.neutral_timer.cancel()

    def timer_callback(self):
        self.delay_cnt += 1
        # delay count decides which movement
        self.get_logger().info(f"delay count: {self.delay_cnt}")
        if (self.delay_cnt < 100):
            # move a little bit more
            twist = Twist()

            twist.linear.x = 0.0  # Do not move a little bit more.
            self.publishCoordinates.publish(twist)

            # Wait for ticks to be 0<ticks<1000

        elif self.delay_cnt < 1000:
            # initialise at default
            arm_cmd = Int16MultiArray()
            arm_cmd.data = [12000 for i in range(12)]
            # set the speed of all the controls
            for i in range(6):
                arm_cmd.data[i+6] = 1000

            # LOWER ARM TO DROP POSITION (KEEP ITEM GRIPPED STILL)

            arm_cmd.data[0] = 11000
            arm_cmd.data[3] = 18000
            arm_cmd.data[2] = 8000
            arm_cmd.data[4] = 6500
            self.arm_pub_.publish(arm_cmd)
            # publish twist
            twist = Twist()
            self.publishCoordinates.publish(twist)

            # Wait for ticks to be 1000<ticks<2000

        elif self.delay_cnt < 2000:
            # initialise at default
            arm_cmd = Int16MultiArray()
            arm_cmd.data = [12000 for i in range(12)]
            # set the speed of all the controls
            for i in range(6):
                arm_cmd.data[i+6] = 1000

            # KEEP ARM IN DROP POSITION
            arm_cmd.data[3] = 18000
            arm_cmd.data[2] = 8000
            arm_cmd.data[4] = 6500
            # open gripper with higher speed (DROP ITEM)
            arm_cmd.data[0+6] = 500
            arm_cmd.data[0] = 0
            self.arm_pub_.publish(arm_cmd)

            # Wait for ticks to be 2000<ticks<3000

        elif self.delay_cnt < 3000:
            # initialise at default
            arm_cmd = Int16MultiArray()
            arm_cmd.data = [12000 for i in range(12)]
            # set the speed of all the controls
            for i in range(6):
                arm_cmd.data[i+6] = 1000
            # set to neutral with closed gripper
            arm_cmd.data[0] = 11000
            arm_cmd.data[3] = 12000
            arm_cmd.data[2] = 12000
            arm_cmd.data[4] = 12000
            self.arm_pub_.publish(arm_cmd)
            # set twist to zero
            twist = Twist()
            twist.angular.z = 0.0
            twist.linear.x = 0.0
            self.publishCoordinates.publish(twist)
        else:
            self.timer.cancel()

    def target_theta_callback(self, msg: MarkerArray):
        twist = Twist()
        KP = -2.5

        # first sequence
        if self.has_set_neutral == False:
            return

        if self.target_reached:
            return

        for marker in msg.markers:
            # hardcoded marker
            if marker.id == 1:
                x = marker.pose.pose.position.x
                y = marker.pose.pose.position.y
                z = marker.pose.pose.position.z
                dist = math.sqrt(x*x+y*y+z*z)
                self.get_logger().info(
                    f"distance to aruco marker: {dist}")
                # when far away
                if (dist > 0.25):

                    theta = math.atan2(x, z)
                    self.delay_cnt = 0
                    twist.angular.z = theta * KP

                    twist.linear.x = 0.15
                    self.publishCoordinates.publish(twist)
                # when close
                else:
                    self.target_reached = True
                    self.timer.reset()


def main():
    rclpy.init()
    node = ArucoDetectMove()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
