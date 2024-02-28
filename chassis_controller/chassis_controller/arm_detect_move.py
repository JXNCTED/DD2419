"""A node for controlling the odometry from detecting an object in the arm camera. """
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Float32MultiArray


class ArmDetectMove(Node):
    def __init__(self):
        super().__init__('arm_detect_move')

        self.theta_linear_sub = self.create_subscription(
            Float32,
            '/arm/camera/object/theta_linear',
            self.target_theta_callback,
            10
        )

        self.publishCoordinates = self.create_publisher(
            Twist,
            '/motor_controller/twist',
            10
        )

    def target_theta_callback(self, msg: Float32MultiArray):
        KP = -2.0
        twist = Twist()
        self.get_logger().info(f"set w: {msg.data * KP}")

        twist.angular.z = msg.data * KP
        twist.linear.x = 0.2

        self.publishCoordinates.publish(twist)


def main():
    rclpy.init()
    node = ArmDetectMove()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
