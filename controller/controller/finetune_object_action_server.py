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
from threading import Lock
from rclpy.callback_groups import ReentrantCallbackGroup


class FinetuneObjectActionServer(Node):
    def __init__(self):
        super().__init__('finetune_object_action_server')
        self.detected_obj_lock = Lock()
        self.target_obj_id = None
        self.detected_pos = None
        self.index = None

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
        self.last_valid_measurement_stamp = None

        self.rate = self.create_rate(100)

        cb_group = ReentrantCallbackGroup()

        self.coeffs_arm = np.array(
            [-0.474424, 0.207336, -0.002361, 0.000427, 0.000000])

        self.arm_detected_obj_sub = self.create_subscription(
            Float32MultiArray, '/detection_ml/arm_bounding_box', self.arm_detected_obj_callback, 10, callback_group=cb_group
        )

        self.arm_camera_image_sub = self.create_subscription(
            Image, '/image_raw', self.arm_camera_image_callback, 10, callback_group=cb_group)

        self.twist_pub = self.create_publisher(
            Twist, '/motor_controller/twist', 10, callback_group=cb_group)

        self.action_server_ = ActionServer(
            self, Finetune, 'finetune', execute_callback=self.execute_callback, goal_callback=self.goal_callback, cancel_callback=self.cancel_callback, callback_group=cb_group
        )

    def arm_detected_obj_callback(self, msg: Float32MultiArray):
        self.last_valid_measurement_stamp = self.get_clock().now()
        self.detected_pos = msg.data
        with self.detected_obj_lock:
            if self.target_obj_id is None:
                return
            self.index = None
            for i in range(0, len(msg.data), 6):
                if int(msg.data[i+5]) == int(self.target_obj_id):
                    self.index = i
                    break
            if self.index is None:
                return
        x, y, w, h = msg.data[self.index:self.index+4]
        center_x = x + w / 2
        center_y = y + h / 2
        self.filter.predict()
        self.filter.update([center_x, center_y])

    def arm_camera_image_callback(self, msg: Image):
        img = CvBridge().imgmsg_to_cv2(msg, 'bgr8')
        self.img = cv2.undistort(img, self.K_arm, self.coeffs_arm)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        result = Finetune.Result()

        # use chassis to move the robot to the target object
        cnt = 0
        TIMEOUT = 200
        while (cnt < TIMEOUT):
            if self.target_obj_id is None:
                self.get_logger().warn('No target object id')
                goal_handle.abort()
                return result

            self.index = None
            if self.detected_pos is None:
                cnt += 1
                self.rate.sleep()
                continue
            for i in range(0, len(self.detected_pos), 6):
                if int(self.detected_pos[i+5]) == int(self.target_obj_id):
                    self.index = i
                    break
            if self.index is None:
                cnt += 1
                self.rate.sleep()
                continue
            else:
                # reset the kalman filter state
                self.filter.x = np.array(
                    [self.detected_pos[self.index], 0, self.detected_pos[self.index+2], 0])
                break
        else:
            self.get_logger().warn('No valid measurement')
            goal_handle.abort()
            return result

        while rclpy.ok():
            # find the object with the target id in field x, y, w, h, confidence, category

            # calculate the center point

            # debug display
            if self.last_valid_measurement_stamp is None:
                continue
            if self.get_clock().now().nanoseconds - self.last_valid_measurement_stamp.nanoseconds > 1e9:
                self.get_logger().warn('No valid measurement. timeout')
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.twist_pub.publish(twist)
                result.success = False
                goal_handle.abort()
                return result
            if self.img is None:
                self.get_logger().warn('No image received')
                goal_handle.abort()
                return result
            display_img = self.img.copy()
            cv2.circle(display_img, (int(self.filter.x[0]), int(self.filter.x[2])),
                       5, (255, 0, 0), -1)

            CENTER = (320, 300)  # center camera coordinate

            cv2.circle(display_img, CENTER, 5, (0, 255, 255), -1)
            # cv2.imshow('image', display_img)
            # cv2.waitKey(1)

            if abs(self.filter.x[0] - CENTER[0]) < 20 and abs(self.filter.x[2] - CENTER[1]) < 40:
                self.get_logger().info('reached')
                break
            theta_normalized = atan2(
                CENTER[1] - self.filter.x[2], CENTER[0] - self.filter.x[0]) / pi

            twist = Twist()
            KP = -1.0
            LINEAR_VEL = 0.06

            if (0 < theta_normalized <= 0.4):
                twist.linear.x = LINEAR_VEL
                twist.angular.z = -(0.5 - theta_normalized) * KP
            elif (0.6 < theta_normalized <= 1):
                twist.linear.x = LINEAR_VEL
                twist.angular.z = (theta_normalized - 0.5) * KP
            elif (0.4 < theta_normalized <= 0.6):
                twist.linear.x = LINEAR_VEL
                twist.angular.z = 0.0
            elif (-0.6 < theta_normalized <= -0.4):
                twist.linear.x = -LINEAR_VEL
                twist.angular.z = 0.0
            elif (-0.4 < theta_normalized <= 0):
                twist.linear.x = -LINEAR_VEL
                twist.angular.z = 0.0
            elif (-1 < theta_normalized <= -0.6):
                twist.linear.x = -LINEAR_VEL
                twist.angular.z = 0.0

            goal_handle.distance = sqrt(
                (self.filter.x[0] - CENTER[0])**2 + (self.filter.x[2] - CENTER[1])**2)

            self.twist_pub.publish(twist)

        twist = Twist()
        for _ in range(100):
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.twist_pub.publish(twist)

        world_z = 0.2  # 20 cm camera above the ground

        # calculate the world coordinate
        world_x = (self.filter.x[0] - CENTER[0]) / self.K_arm[0, 0] * world_z
        world_y = (self.filter.x[2] - CENTER[1]) / self.K_arm[1, 1] * world_z

        img = cv2.bilateralFilter(self.img, 25, 90, 70)

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150, apertureSize=3, L2gradient=True)

        contours, _ = cv2.findContours(
            edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # find contours only within 180 pixels from the center
        valid_contours = []
        if len(contours) > 0:
            length_mask = [cv2.arcLength(c, True) > 100 for c in contours]
            dist_mask = [abs(cv2.pointPolygonTest(
                c, (self.filter.x[0], self.filter.x[2]), True)) < 70 for c in contours]

            valid_contours = [c for c, a, b in zip(
                contours, length_mask, dist_mask) if a and b]

        if len(valid_contours) == 0:
            self.get_logger().warn('No valid contours found')
            goal_handle.abort()
            return result

        rect = cv2.minAreaRect(np.concatenate(valid_contours))
        cv2.drawContours(img, valid_contours, -1, (0, 255, 0), 2)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        # cv2.drawContours(img, [box], 0, (0, 0, 255), 2)
        # cv2.circle(img, (int(self.filter.x[0]), int(self.filter.x[2])),
        #            5, (255, 0, 0), -1)
        # cv2.circle(img, CENTER, 5, (0, 255, 255), -1)
        # cv2.imshow('image', img)

        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        self.get_logger().info(
            f'world_x: {world_x}, world_y: {world_y}, angle: {rect[2]}')

        self.get_logger().info('Finetune object succeeded')
        result.success = True
        result.position = [world_x, world_y]  # not sure if this is correct
        result.angle = rect[2] / 180.0 * pi

        goal_handle.succeed()
        return result

    def goal_callback(self, goal_request):
        with self.detected_obj_lock:
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
