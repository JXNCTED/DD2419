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
from threading import Lock
# from viztracer import VizTracer


super_cls_dict = {
    0: "none",
    1: "cube",
    2: "animal",
    3: "none",
    4: "sphere",
    5: "cube",
    6: "sphere",
    7: "animal",
    8: "animal",
    9: "animal",
    10: "animal",
    11: "cube",
    12: "sphere",
    13: "animal",
    14: "cube",
}


class FinetuneObjectActionServer(Node):
    def __init__(self):
        super().__init__('finetune_object_action_server')
        self.detected_obj_lock = Lock()
        # self.target_obj_id = None
        self.target_super_cls = "none"
        self.detected_pos = None
        self.index = None

        self.K_arm = np.array([[513.34301, 0., 307.89617],
                               [0., 513.84807, 244.62007],
                               [0., 0., 1.]])

        self.img = None

        self.running = False

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
        self.wiggle_rate = self.create_rate(5)

        self.lock = Lock()

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
        
        self.fine_tune_pub = self.create_publisher(
            Image, '/finetune', 10)

        self.action_server_ = ActionServer(
            self, Finetune, 'finetune', execute_callback=self.execute_callback, goal_callback=self.goal_callback, cancel_callback=self.cancel_callback, callback_group=cb_group
        )

    def arm_detected_obj_callback(self, msg: Float32MultiArray):
        if not self.running:
            return
        self.last_valid_measurement_stamp = self.get_clock().now()
        self.detected_pos = msg.data
        with self.detected_obj_lock:
            if self.target_super_cls == 'none':
                return
            self.index = None
            for i in range(0, len(msg.data), 6):
                # if int(msg.data[i+5]) == int(self.target_obj_id):
                if super_cls_dict[int(msg.data[i+5])] == self.target_super_cls:
                    self.index = i
                    break
            if self.index is None:
                self.filter.predict()
                return
        x, y, w, h = msg.data[self.index:self.index+4]
        center_x = x + w / 2
        center_y = y + h / 2
        self.filter.predict()
        self.filter.update([center_x, center_y])

    def arm_camera_image_callback(self, msg: Image):
        if not self.running:
            return
        img = CvBridge().imgmsg_to_cv2(msg, 'bgr8')
        self.img = cv2.undistort(img, self.K_arm, self.coeffs_arm)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        self.running = True
        result = Finetune.Result()

        cnt = 0
        CENTER = (320, 300)  # center camera coordinate
        THRES = (80, 60)
        while rclpy.ok():
            if (self.index is None or self.get_clock().now().nanoseconds - self.last_valid_measurement_stamp.nanoseconds > 1e9):
                self.get_logger().warn('No valid measurement, wiggling...')
                cnt += 1
                self.wiggle_rate.sleep()
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = 0.1
                self.twist_pub.publish(twist)
                self.wiggle_rate.sleep()
                twist.angular.z = -0.1
                self.twist_pub.publish(twist)
                self.wiggle_rate.sleep()
                twist.angular.z = 0.0
                self.twist_pub.publish(twist)
                if (cnt > 10):
                    self.get_logger().warn('No valid measurement. timeout')
                    twist = Twist()
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.twist_pub.publish(twist)
                    result.success = False
                    goal_handle.abort()
                    self.running = False
                    return result
                continue
            else:
                cnt = 0

            display_img = self.img.copy()
            cv2.circle(display_img, (int(self.filter.x[0]), int(self.filter.x[2])),
                       5, (255, 0, 0), -1)

            cv2.circle(display_img, CENTER, 5, (0, 255, 255), -1)
            cv2.rectangle(display_img, (int(CENTER[0] - THRES[0]), int(CENTER[1] - THRES[1])),
                          (int(CENTER[0] + THRES[0]), int(CENTER[1] + THRES[1])), (0, 255, 255), 2)
            self.fine_tune_pub.publish(CvBridge().cv2_to_imgmsg(display_img))

            if abs(self.filter.x[0] - CENTER[0]) < THRES[0] and abs(self.filter.x[2] - CENTER[1]) < THRES[1]:
                self.get_logger().info(
                    f'\033[92mReached!\033[0m')
                break
            theta_normalized = atan2(
                CENTER[1] - self.filter.x[2], CENTER[0] - self.filter.x[0]) / pi

            twist = Twist()
            KP = -0.6
            LINEAR_VEL = 0.05

            if (0 < theta_normalized <= 0.25):
                twist.linear.x = -LINEAR_VEL
                twist.angular.z = -(0.5 - theta_normalized) * KP
            elif (0.75 < theta_normalized <= 1):
                twist.linear.x = -LINEAR_VEL
                twist.angular.z = (theta_normalized - 0.5) * KP
            elif (0.25 < theta_normalized <= 0.75):
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
        for _ in range(10):
            display_img = self.img.copy()
            cv2.circle(display_img, (int(self.filter.x[0]), int(self.filter.x[2])),
                       5, (255, 0, 0), -1)
            cv2.rectangle(display_img, (int(CENTER[0] - THRES[0]), int(CENTER[1] - THRES[1])),
                          (int(CENTER[0] + THRES[0]), int(CENTER[1] + THRES[1])), (0, 255, 255), 2)
            self.fine_tune_pub.publish(CvBridge().cv2_to_imgmsg(display_img))
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.twist_pub.publish(twist)

            self.wiggle_rate.sleep()

        world_z = 0.2  # 20 cm camera above the ground

        # calculate the world coordinate
        world_x = (self.filter.x[0] - 320) / self.K_arm[0, 0] * world_z
        world_y = (self.filter.x[2] - 240) / self.K_arm[1, 1] * world_z

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
                c, (self.filter.x[0], self.filter.x[2]), True)) < 50 for c in contours]

            valid_contours = [c for c, a, b in zip(
                contours, length_mask, dist_mask) if a and b]

        if len(valid_contours) == 0:
            self.get_logger().warn('No valid contours found')
            goal_handle.abort()
            self.running = False
            return result

        rect = cv2.minAreaRect(np.concatenate(valid_contours))
        cv2.drawContours(img, valid_contours, -1, (0, 255, 0), 2)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        cv2.drawContours(img, [box], 0, (0, 0, 255), 2)
        cv2.circle(img, (int(self.filter.x[0]), int(self.filter.x[2])),
                   5, (255, 0, 0), -1)
        cv2.circle(img, CENTER, 5, (0, 255, 255), -1)
        cv2.rectangle(display_img, (int(CENTER[0] - THRES[0]), int(CENTER[1] - THRES[1])),
                          (int(CENTER[0] + THRES[0]), int(CENTER[1] + THRES[1])), (0, 255, 255), 2)
        self.fine_tune_pub.publish(CvBridge().cv2_to_imgmsg(img))
        self.get_logger().info(
            f'world_x: {world_x}, world_y: {world_y}, angle: {rect[2]}')

        self.get_logger().info('\033[92mFinetune succeeded\033[0m')
        result.success = True
        result.position = [world_x, world_y]  # not sure if this is correct
        result.angle = rect[2] / 180.0 * pi

        self.filter.x = np.array([0, 0, 0, 0])
        self.filter.predict()

        goal_handle.succeed()
        self.running = False
        return result

    def goal_callback(self, goal_request):
        with self.detected_obj_lock:
            # self.target_obj_id = goal_request.object_id
            self.target_super_cls = goal_request.super_category
            self.get_logger().info(
                f'Received goal request: {goal_request.super_category}')
        return rclpy.action.server.GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return rclpy.action.server.CancelResponse.ACCEPT

    def destroy(self):
        self.get_logger().info('Destroying finetune object action server...')
        self.action_server_.destroy()
        super().destroy_node()


def main():
    # tracer = VizTracer()
    # tracer.start()
    rclpy.init()
    executor = rclpy.executors.MultiThreadedExecutor()
    finetune_object_action_server = FinetuneObjectActionServer()
    try:
        executor.add_node(finetune_object_action_server)
        while executor.context.ok():
            executor.spin_once()
        # rclpy.spin(finetune_object_action_server, executor)
    except KeyboardInterrupt:
        pass

    # print("stop tracing...")
    # tracer.stop()
    # tracer.save('finetune_object_action_server.html')

    rclpy.shutdown()

