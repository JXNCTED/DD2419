#!/usr/bin/env python
import rclpy
import rclpy.action
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import JointState
from rclpy.action import ActionServer
from robp_interfaces.action import Arm
from math import pi, sin, cos, atan2, sqrt, acos, asin


def j12_ik(x, y):
    """
    joint 1 and 2 inverse kinematics
    """
    L1 = 0.101
    L2 = 0.094

    d = sqrt(x**2 + y**2)

    if d > L1 + L2:
        return 0, 0

    beta = acos((L1**2 + L2**2 - d**2) / (2 * L1 * L2))
    gamma = atan2(y, x)
    alpha = acos((L1**2 + d**2 - L2**2) / (2 * L1 * d))

    q1 = pi / 2 - gamma - alpha
    q2 = pi - beta

    return q1, q2


class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')

        self.current_joint_pos = [12000, 12000, 12000, 12000, 12000, 12000]
        self.command_list = [12000 for _ in range(6)]

        self.current_command = "none"

        self.arm_pub_ = self.create_publisher(
            Int16MultiArray, '/multi_servo_cmd_sub', 10)
        self.arm_pos_sub_ = self.create_subscription(
            JointState, '/servo_pos_publisher', self.arm_pos_callback, 10)

        self.action_server_ = ActionServer(
            self, Arm, 'arm', execute_callback=self.execute_callback, goal_callback=self.goal_callback, cancel_callback=self.cancel_callback
        )

    def arm_pos_callback(self, msg: JointState):
        self.current_joint_pos = msg.position

    def execute_callback(self, goal_handle) -> Arm.Result:
        self.get_logger().info(f'executing {self.current_command}')

        dummyx = 0.15
        dummyy = 0.00
        if self.current_command == "pick":
            q1, q2 = j12_ik(dummyx, dummyy)
            if q1 == 0 and q2 == 0:
                result = Arm.Result()
                result.success = False
                goal_handle.abort()
                return result

            self.command_list[0] = -1
            self.command_list[1] = -1
            self.command_list[2] = -1
            self.command_list[3] = int(q2 * 180 / pi * 100) + 12000
            self.command_list[4] = 12000 - int(q1 * 180 / pi * 100)
            self.command_list[5] = -1

            move_time = [1000 for _ in range(6)]

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
            self.get_logger().info('Accepting the goal request')
            return rclpy.action.GoalResponse.ACCEPT
        elif goal_request.command == "place":
            self.current_command = goal_request.command
            self.get_logger().info('Accepting the goal request')
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
    node = ArmController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
