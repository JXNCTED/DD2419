#!/usr/bin/env python
import rclpy
import rclpy.action
from rclpy.node import Node
from rclpy.action import ActionServer
from robp_interfaces.action import Finetune


class FinetuneObjectActionServer(Node):
    def __init__(self):
        super().__init__('finetune_object_action_server')

        self.action_server_ = ActionServer(
            self, Finetune, 'finetune', execute_callback=self.execute_callback, goal_callback=self.goal_callback, cancel_callback=self.cancel_callback
        )

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        self.get_logger().info('Goal succeeded!')
        goal_handle.succeed()

        return Finetune.Result()

    def goal_callback(self, goal_request):
        self.get_logger().info('Received goal request')
        return rclpy.action.server.GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return rclpy.action.server.CancelResponse.ACCEPT

    def destroy(self):
        self.get_logger().info('Destroying finetune object action server...')
        self.action_server_.destroy()
        super().destroy_node()


def main():
    rclpy.init()
    executor = rclpy.executors.MultiThreadedExecutor(3)
    finetune_object_action_server = FinetuneObjectActionServer()

    try:
        rclpy.spin(finetune_object_action_server, executor)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
