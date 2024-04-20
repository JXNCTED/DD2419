import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3

# We need to..

# listen to the topic that detection publishes to
# Get the coordinate from the center of that pointcloud.
# Set the cartesian controller coordinate to that controller.


class ObjectDetectMove(Node):
    def __init__(self):
        super().__init__('obj_detect_move')
        self.target_reached = False
        self.subscription = self.create_subscription(
            Vector3,
            '/camera/depth/color/detected_pos',
            self.target_pose_callback,
            10
        )
        self.publishCoordinates = self.create_publisher(
            Twist,
            '/motor_controller/twist',
            10
        )
        self.publishArmStatus = self.create_publisher(
            Bool,
            '/dist_bool',
            10
        )

    def target_pose_callback(self, msg: Vector3):
        # KP = -2.0
        # twist = Twist()
        # self.get_logger().info(f"set w: {msg.data * KP}")

        # twist.angular.z = msg.data * KP
        # twist.linear.x = 0.5

        # self.publishCoordinates.publish(twist)
        # use vector3, x is yaw and y is distance
        if msg.y > 0.24 and not self.target_reached:
            KP = -2.0
            twist = Twist()
            self.get_logger().info(f"dist: {msg.y}, set w: {msg.x * KP}")

            twist.angular.z = msg.x * KP
            twist.linear.x = 0.5
            self.publishCoordinates.publish(twist)
        else:
            self.target_reached = True
            dist_msg = Bool()
            dist_msg.data = True
            self.publishArmStatus.publish(dist_msg)
            return


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
