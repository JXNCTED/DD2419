import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
# from sensor_msgs.msg import Joy
from std_msgs.msg import Float32
# include "sensor_msgs/msg/point_cloud2.hpp"
# from sensor_msgs.msg import PointCloud2

# We need to..

# listen to the topic that detection publishes to
# Get the coordinate from the center of that pointcloud.
# Set the cartesian controller coordinate to that controller.


class ObjectDetectMove(Node):
    def __init__(self):
        super().__init__('obj_detect_move')
        self.subscription = self.create_subscription(
            Float32,
            '/camera/depth/color/target_theta',
            self.target_theta_callback,
            10
        )
        self.publishCoordinates = self.create_publisher(
            Twist,
            '/motor_controller/twist',
            10
        )

    def target_theta_callback(self, msg: Float32):
        KP = -2.0
        twist = Twist()
        self.get_logger().info(f"set w: {msg.data * KP}")

        twist.angular.z = msg.data * KP
        twist.linear.x = 0.5

        self.publishCoordinates.publish(twist)


def main():
    rclpy.init()
    node = ObjectDetectMove()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
