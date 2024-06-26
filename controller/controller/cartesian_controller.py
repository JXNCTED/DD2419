#!/usr/bin/env python
import rclpy
from rclpy.node import Node

from robp_interfaces.msg import DutyCycles, Encoders
from geometry_msgs.msg import Twist

from math import pi


class CartesianController(Node):
    def __init__(self):
        super().__init__('cartesian_controller')
        self.target_w = 0.0
        self.target_v = 0.0

        self.int_error_wheel1 = 0.0
        self.int_error_wheel2 = 0.0

        self.last_delta_left = 0.0
        self.last_delta_right = 0.0

        self.twist_sub_ = self.create_subscription(
            Twist, '/motor_controller/twist', self.twist_callback, 10)
        self.encoder_sub_ = self.create_subscription(
            Encoders, '/motor/encoders', self.encoder_callback, 10)
        self.duty_cycle_pub_ = self.create_publisher(
            DutyCycles, '/motor/duty_cycles', 10)

    def twist_callback(self, msg):
        self.target_v = msg.linear.x
        self.target_w = msg.angular.z

    def encoder_callback(self, msg):
        TICK_PER_REV = 3600
        WHEEL_RADIUS = 0.0352
        WHEEL_BASE = 0.23

        KP = 0.8
        KI = 0.8

        DT = 50 / 1000
        ALPHA = 0.5

        self.last_delta_left = msg.delta_encoder_left * \
            ALPHA + self.last_delta_left * (1 - ALPHA)
        self.last_delta_right = msg.delta_encoder_right * \
            ALPHA + self.last_delta_right * (1 - ALPHA)

        vw1 = (self.last_delta_left * 2 * pi) / \
            TICK_PER_REV * WHEEL_RADIUS / DT
        vw2 = (self.last_delta_right * 2 * pi) / \
            TICK_PER_REV * WHEEL_RADIUS / DT

        target_wheel1 = (self.target_v - self.target_w * WHEEL_BASE)
        target_wheel2 = (self.target_v + self.target_w * WHEEL_BASE)

        error_wheel1 = target_wheel1 - vw1
        error_wheel2 = target_wheel2 - vw2

        self.int_error_wheel1 += error_wheel1 * DT
        self.int_error_wheel2 += error_wheel2 * DT

        self.int_error_wheel1 = min(max(self.int_error_wheel1, -0.1), 0.1)
        self.int_error_wheel2 = min(max(self.int_error_wheel2, -0.1), 0.1)

        pwm_1 = min(max(KP * error_wheel1 + KI *
                    self.int_error_wheel1, -1.0), 1.0)
        pwm_2 = min(max(KP * error_wheel2 + KI *
                    self.int_error_wheel2, -1.0), 1.0)

        # self.get_logger().info(f"pwm: {pwm_1} {pwm_2}")
        # self.get_logger().info(f"error: {error_wheel1} {error_wheel2}")

        pub_msg = DutyCycles()
        pub_msg.duty_cycle_left = pwm_1
        pub_msg.duty_cycle_right = pwm_2

        self.duty_cycle_pub_.publish(pub_msg)


def main():
    rclpy.init()
    node = CartesianController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
