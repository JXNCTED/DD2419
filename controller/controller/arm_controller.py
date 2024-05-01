#!/usr/bin/env python
import rclpy
import rclpy.action
import rclpy.logging
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import JointState
from rclpy.action import ActionServer
from robp_interfaces.action import Arm
from math import pi, sin, cos, atan2, sqrt, acos, asin

from numpy import clip


def j12_ik(x, z):
    """
    joint 1 and 2 inverse kinematics
    """
    L1 = 0.101
    L2 = 0.094

    d = sqrt(x**2 + z**2)

    if d > L1 + L2:  #
        rclpy.logging.get_logger("ik").warn(
            f"invalid position, deacrease d to {L1 + L2}")
        d = L1 + L2

    # beta = acos((L1**2 + L2**2 - d**2) / (2 * L1 * L2))
    # gamma = atan2(z, x)
    # alpha = acos((L1**2 + d**2 - L2**2) / (2 * L1 * d))

    beta = acos(clip((L1**2 + L2**2 - d**2) / (2 * L1 * L2), -1, 1))
    gamma = atan2(z, x)
    alpha = acos(clip((L1**2 + d**2 - L2**2) / (2 * L1 * d), -1, 1))

    q1 = pi / 2 - gamma - alpha
    q2 = pi - beta

    q3 = alpha + beta + gamma - pi / 2

    return q1, q2, q3


class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')

        self.current_joint_pos = [12000, 12000, 12000, 12000, 12000, 12000]
        self.command_list = [12000 for _ in range(6)]

        self.current_command = "none"

        self.object_angle = 0.0
        self.object_position = [0.0, 0.0]

        self.arm_pub_ = self.create_publisher(
            Int16MultiArray, '/multi_servo_cmd_sub', 10)
        self.arm_pos_sub_ = self.create_subscription(
            JointState, '/servo_pos_publisher', self.arm_pos_callback, 10)

        self.rate = self.create_rate(1/3)
        self.rate_place = self.create_rate(1/4)  # sleep

        self.action_server_ = ActionServer(
            self, Arm, 'arm', execute_callback=self.execute_callback, goal_callback=self.goal_callback, cancel_callback=self.cancel_callback
        )

    def arm_pos_callback(self, msg: JointState):
        self.current_joint_pos = msg.position

    def execute_callback(self, goal_handle) -> Arm.Result:
        self.get_logger().info(f'executing {self.current_command}')

        if self.current_command == "pick":
            # open the gripper
            self.command_list[0] = 0
            for i in range(5):
                self.command_list[i+1] = -1

            msg = Int16MultiArray()
            msg.data.extend(self.command_list)
            msg.data.extend([1000 for _ in range(6)])

            self.arm_pub_.publish(msg)

            self.rate.sleep()

            z = 0.02  # compensate for the gravity

            x, y = self.object_position
            angle = self.object_angle
            q1, q2, q3 = j12_ik(sqrt(x**2 + y**2), z)
            if q1 == 0 and q2 == 0 and q3 == 0:
                self.get_logger().warn("Invalid position")
                result = Arm.Result()
                result.success = False
                goal_handle.abort()
                return result

            self.command_list[0] = -1
            self.command_list[1] = 12000 - \
                int((angle - atan2(x, y)) * 18000 / pi)
            self.command_list[2] = 12000 - int(q3 * 18000 / pi)
            self.command_list[3] = 12000 + int(q2 * 18000 / pi)
            self.command_list[4] = 12000 - int(q1 * 18000 / pi)
            self.command_list[5] = 12000 + int(atan2(x, y) * 18000 / pi)

            for i in range(6):
                if self.command_list[i] >= 24000:
                    self.command_list[i] = 24000
                elif self.command_list[i] <= -1:
                    self.command_list[i] = -1

            move_time = [1000 for _ in range(6)]

            msg = Int16MultiArray()
            msg.data.extend(self.command_list)
            msg.data.extend(move_time)

            self.arm_pub_.publish(msg)

            self.rate.sleep()

            # now close the gripper
            self.command_list[0] = 16000  # or whatever pick position is

            for i in range(5):
                self.command_list[i+1] = -1

            msg = Int16MultiArray()
            msg.data.extend(self.command_list)
            msg.data.extend(move_time)

            self.arm_pub_.publish(msg)

            self.rate.sleep()

            for i in range(5):
                self.command_list[i+1] = 12000

            msg = Int16MultiArray()
            msg.data.extend(self.command_list)
            msg.data.extend(move_time)

            self.arm_pub_.publish(msg)

            self.get_logger().info("\033[92mPicked the object\033[0m")

        elif self.current_command == "place":
            self.rate_place.sleep()
            z = 0.05

            x, y = 0, 0.15
            angle = 0
            q1, q2, q3 = j12_ik(sqrt(x**2 + y**2), z)
            if q1 == 0 and q2 == 0 and q3 == 0:
                self.get_logger().warn("Invalid position")
                result = Arm.Result()
                result.success = False
                goal_handle.abort()
                return result

            self.command_list[0] = -1
            self.command_list[1] = 12000 - \
                int((angle - atan2(x, y)) * 18000 / pi)
            self.command_list[2] = 12000 - \
                int(q3 * 18000 / pi) + int(pi / 2 * 18000 / pi)
            self.command_list[3] = 12000 + int(q2 * 18000 / pi)
            self.command_list[4] = 12000 - int(q1 * 18000 / pi)
            self.command_list[5] = 12000 + int(atan2(x, y) * 18000 / pi)

            for i in range(6):
                if self.command_list[i] >= 24000:
                    self.command_list[i] = 24000
                elif self.command_list[i] <= -1:
                    self.command_list[i] = -1

            move_time = [1000 for _ in range(6)]

            msg = Int16MultiArray()
            msg.data.extend(self.command_list)
            msg.data.extend(move_time)

            self.arm_pub_.publish(msg)

            self.rate_place.sleep()

            # now open the gripper
            self.command_list[0] = 0

            for i in range(5):
                self.command_list[i+1] = -1

            msg = Int16MultiArray()
            msg.data.extend(self.command_list)
            msg.data.extend(move_time)

            self.arm_pub_.publish(msg)

            self.rate_place.sleep()

            for i in range(6):
                self.command_list[i] = 12000

            msg = Int16MultiArray()
            msg.data.extend(self.command_list)
            msg.data.extend(move_time)

            self.arm_pub_.publish(msg)

        result = Arm.Result()
        result.success = True

        goal_handle.succeed()

        return result

    def goal_callback(self, goal_request):
        self.get_logger().info(
            f'Received goal request: {goal_request.command}')

        if goal_request.command == "pick":
            self.current_command = goal_request.command
            self.object_angle = goal_request.angle
            self.object_position = goal_request.position
            if (self.object_position[1] < 0.14):
                self.get_logger().warn("increase y position")
                self.object_position[1] = 0.14
            self.get_logger().info(
                f'Accepting {goal_request.command, goal_request.angle, goal_request.position}')
            return rclpy.action.GoalResponse.ACCEPT
        elif goal_request.command == "place":
            self.current_command = goal_request.command
            self.get_logger().info(f'Accepting {goal_request.command}')
            return rclpy.action.GoalResponse.ACCEPT
        else:
            self.get_logger().info('Rejecting the goal request')
            return rclpy.action.GoalResponse.REJECT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return rclpy.action.CancelResponse.ACCEPT

    def destroy(self):
        self.get_logger().info('Destroying arm controller...')
        self.action_server_.destroy()
        super().destroy_node()


def main():
    rclpy.init()
    executor = rclpy.executors.MultiThreadedExecutor(3)
    node = ArmController()
    try:
        rclpy.spin(node, executor)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
