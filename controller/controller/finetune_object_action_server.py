#!/usr/bin/env python
import rclpy
import rclpy.action
from rclpy.node import Node
from rclpy.action import ActionServer
from robp_interfaces.action import Finetune
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from math import atan2, pi, sqrt
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np


class FinetuneObjectActionServer(Node):
    def __init__(self):
        super().__init__('finetune_object_action_server')

        self.detected_obj = None
        self.target_obj_id = None

        self.K_arm = np.array([[513.34301, 0., 307.89617],
                               [0., 513.84807, 244.62007],
                               [0., 0., 1.]])

        self.img = None

        self.coeffs_arm = np.array(
            [-0.474424, 0.207336, -0.002361, 0.000427, 0.000000])

        self.arm_detected_obj_sub = self.create_subscription(
            Float32MultiArray, '/detection_ml/arm_bounding_box', self.arm_detected_obj_callback, 10
        )

        self.arm_camera_image_sub = self.create_subscription(
            Image, '/raw_img', self.arm_camera_image_callback, 10)

        self.twist_pub = self.create_publisher(
            Twist, '/motor_controller/twist', 10)

        self.action_server_ = ActionServer(
            self, Finetune, 'finetune', execute_callback=self.execute_callback, goal_callback=self.goal_callback, cancel_callback=self.cancel_callback
        )

    def arm_detected_obj_callback(self, msg: Float32MultiArray):
        self.detected_obj = msg

    def arm_camera_image_callback(self, msg: Image):
        self.img = CvBridge().imgmsg_to_cv2(msg, 'bgr8')
        self.img = cv2.undistort(self.img, self.K_arm, self.coeffs_arm)

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
                CENTER[1] - center_y, CENTER[0] - center_x) / pi

            self.get_logger().info(
                f'theta_normalized: {theta_normalized}, center: {center_x, center_y}, target: {CENTER}')
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

            self.twist_pub.publish(twist)

        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.twist_pub.publish(twist)

        world_z = 0.2  # 20 cm camera above the ground

        world_x = (center_x - self.K_arm[0, 2]) * world_z / self.K_arm[0, 0]
        world_y = (center_y - self.K_arm[1, 2]) * world_z / self.K_arm[1, 1]

        result.success = True
        result.position = [world_x, world_y]
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
