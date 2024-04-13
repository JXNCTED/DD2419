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
from filterpy.kalman import KalmanFilter


class FinetuneObjectActionServer(Node):
    def __init__(self):
        super().__init__('finetune_object_action_server')

        self.detected_obj = None
        self.target_obj_id = None

        self.K_arm = np.array([[513.34301, 0., 307.89617],
                               [0., 513.84807, 244.62007],
                               [0., 0., 1.]])

        self.img = None

        self.filter = KalmanFilter(dim_x=4, dim_z=2)
        self.filter.x = np.array([0, 0, 0, 0])  # x, dx, y, dy
        self.filter.F = np.array([[1, 1, 0, 0],
                                  [0, 1, 0, 0],
                                  [0, 0, 1, 1],
                                  [0, 0, 0, 1]])
        self.filter.H = np.array([[1, 0, 0, 0],
                                  [0, 0, 1, 0]])
        self.filter.P *= 500
        self.filter.R = np.array([[0.5, 0],
                                  [0, 0.5]])
        self.filter.Q = np.array([[0.1, 0, 0, 0],
                                  [0, 0.1, 0, 0],
                                  [0, 0, 0.1, 0],
                                  [0, 0, 0, 0.1]])
        self.index = None
        self.last_valid_measurement_stamp = None

        self.coeffs_arm = np.array(
            [-0.474424, 0.207336, -0.002361, 0.000427, 0.000000])

        self.arm_detected_obj_sub = self.create_subscription(
            Float32MultiArray, '/detection_ml/arm_bounding_box', self.arm_detected_obj_callback, 10
        )

        self.arm_camera_image_sub = self.create_subscription(
            Image, '/image_raw', self.arm_camera_image_callback, 10)

        self.twist_pub = self.create_publisher(
            Twist, '/motor_controller/twist', 10)

        self.action_server_ = ActionServer(
            self, Finetune, 'finetune', execute_callback=self.execute_callback, goal_callback=self.goal_callback, cancel_callback=self.cancel_callback
        )

    def arm_detected_obj_callback(self, msg: Float32MultiArray):
        if self.target_obj_id is None:
            return
        self.index = None
        for i in range(0, len(msg.data), 6):
            if int(msg.data[i+5]) == int(self.target_obj_id):
                self.index = i
                break
        self.detected_obj = self.index if self.index is not None else None
        if self.detected_obj is None:
            return
        x, y, w, h = msg.data[self.index:self.index+4]
        center_x = x + w / 2
        center_y = y + h / 2
        self.last_valid_measurement_stamp = self.get_clock().now()
        self.filter.predict()
        self.filter.update([center_x, center_y])

    def arm_camera_image_callback(self, msg: Image):
        img = CvBridge().imgmsg_to_cv2(msg, 'bgr8')
        self.img = cv2.undistort(img, self.K_arm, self.coeffs_arm)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        result = Finetune.Result()

        if self.detected_obj is None:
            self.get_logger().warn('No object detected')
            goal_handle.abort()
            return result

        # use chassis to move the robot to the target object
        is_first = True
        while rclpy.ok():
            # find the object with the target id in field x, y, w, h, confidence, category

            # calculate the center point

            # debug display
            if self.get_clock().now().nanoseconds - self.last_valid_measurement_stamp.nanoseconds > 5e8:  # 0.5s
                self.get_logger().warn('No valid measurement. timeout')
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.twist_pub.publish(twist)
                goal_handle.abort()
                return result
            display_img = self.img.copy()
            cv2.circle(display_img, (int(self.filter.x[0]), int(self.filter.x[2])),
                       5, (255, 0, 0), -1)

            CENTER = (320, 240)  # center camera coordinate

            cv2.circle(display_img, CENTER, 5, (0, 255, 255), -1)
            cv2.imshow('image', display_img)
            cv2.waitKey(1)

            if abs(self.filter.x[0] - CENTER[0]) < 32 and abs(self.filter.x[2] - CENTER[1]) < 60:
                self.get_logger().info('reached')
                break
            theta_normalized = atan2(
                CENTER[1] - self.filter.x[2], CENTER[0] - self.filter.x[0]) / pi

            self.get_logger().info(f'x{self.filter.x}')

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
                (self.filter.x[0] - CENTER[0])**2 + (self.filter.x[2] - CENTER[1])**2)

            self.twist_pub.publish(twist)

        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.twist_pub.publish(twist)

        world_z = 0.2  # 20 cm camera above the ground

        world_x = self.filter.x[0] * world_z / self.K_arm[0, 0]
        world_y = self.filter.x[2] * world_z / self.K_arm[1, 1]

        result.success = True
        result.position = [world_x, world_y]
        result.angle = 0.0
        cv2.destroyAllWindows()

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
