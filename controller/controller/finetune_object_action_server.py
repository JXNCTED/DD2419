#!/usr/bin/env python
import rclpy
import rclpy.action
from rclpy.node import Node
from rclpy.action import ActionServer
from robp_interfaces.action import Finetune
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from math import atan2, pi, sqrt


class FinetuneObjectActionServer(Node):
    def __init__(self):
        super().__init__('finetune_object_action_server')

        self.detected_obj = None
        self.target_obj_id = None

        self.arm_detected_obj_sub = self.create_subscription(
            Float32MultiArray, '/detection_ml/arm_bounding_box', self.arm_detected_obj_callback, 10
        )

        self.twist_pub = self.create_publisher(
            Twist, '/motor_controller/twist', 10)

        self.action_server_ = ActionServer(
            self, Finetune, 'finetune', execute_callback=self.execute_callback, goal_callback=self.goal_callback, cancel_callback=self.cancel_callback
        )

    def arm_detected_obj_callback(self, msg: Float32MultiArray):
        self.detected_obj = msg

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        result = Finetune.Result()

        if self.detected_obj is None:
            self.get_logger().warn('No object detected')
            goal_handle.abort()
            return result

        # use chassis to move the robot to the target object
        while rclpy.ok():
            # find the object with the target id in field x, y, w, h, confidence, category
            index = None
            for i in range(0, len(self.detected_obj.data), 6):
                if int(self.detected_obj.data[i+5]) == int(self.target_obj_id):
                    index = i
                    break
            if index is None:
                continue

            # calculate the center point
            x, y, w, h = self.detected_obj.data[index:index+4]
            center_x = x + w / 2
            center_y = y + h / 2

            CENTER = (320, 240)  # center camera coordinate
            # continue if the object is in the center of the camera
            if abs(center_x - CENTER[0]) < 32 and abs(center_y - CENTER[1]) < 60:
                self.get_logger().info('reached')
                break
            theta_normalized = atan2(
                center_y - CENTER[1], center_x - CENTER[0]) / pi

            self.get_logger().info(
                f'center_x: {center_x}, center_y: {center_y}')

            twist = Twist()
            KP = -2.0
            if (0 < theta_normalized <= 0.4):
                twist.linear.x = 0.1
                twist.angular.z = -(0.5 - theta_normalized) * KP
            elif (0.6 < theta_normalized <= 1):
                twist.linear.x = 0.1
                twist.angular.z = (theta_normalized - 0.5) * KP
            elif (0.4 < theta_normalized <= 0.6):
                twist.linear.x = 0.1
                twist.angular.z = 0.0
            elif (-0.6 < theta_normalized <= -0.4):
                twist.linear.x = -0.1
                twist.angular.z = 0.0
            elif (-0.4 < theta_normalized <= 0):
                twist.linear.x = -0.1
                twist.angular.z = 0.0
            elif (-1 < theta_normalized <= -0.6):
                twist.linear.x = -0.1
                twist.angular.z = 0.0

            goal_handle.distance = sqrt(
                (center_x - CENTER[0])**2 + (center_y - CENTER[1])**2)

            self.get_logger().info(
                f'linear: {twist.linear.x}, angular: {twist.angular.z}, theta: {theta_normalized}')
            self.twist_pub.publish(twist)

        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.twist_pub.publish(twist)

        result.success = True
        result.position = [0.0, 0.0]
        result.angle = 0.0

        goal_handle.succeed()
        return result

    def goal_callback(self, goal_request):
        self.target_obj_id = goal_request.object_id
        self.get_logger().info(
            f'Received goal request: {goal_request.object_id}')
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
