import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from robp_interfaces.action import Pursuit
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class PursuitActionServer(Node):
    def __init__(self):
        super().__init__("pursuit_action_server")

        self.odom_x = 0
        self.odom_y = 0
        self.odom_yaw = 0

        self.action_server = ActionServer(
            self, Pursuit, "pursuit", self.execute_callback
        )
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )

    # Pseudo code sort of
    def execute_callback(self, goal_handle):
        self.get_logger().info("Executing goal...")

        feedback_msg = Pursuit.Feedback()
        for waypoint in goal_handle.request.waypoints:
            lin, ang, t = self.velocity(waypoint)
            feedback_msg.current_velocity.linear.x = lin
            feedback_msg.current_velocity.angular.z = ang
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(t)
            # while odom.x and y is not close to waypoint stall

        result = Pursuit.Result()
        return result

    # Not implemented yet see test_pure_pursuit in robotics_group7
    def velocity(self, target):
        x = self.odom_x
        y = self.odom_y
        yaw = self.odom_yaw
        dx = target.x - x
        dy = target.y - y
        y_cos = np.cos(yaw)
        y_sin = np.sin(yaw)
        tx = dx * y_cos + dy * y_sin
        ty = -dx * y_sin + dy * y_cos
        dist = np.hypot(tx, ty)
        alpha = np.arctan2(ty, tx)
        radius = dist / (2 * np.sin(alpha))
        lin_v = 1.0
        arc_length = 2.0 * alpha * radius
        time = arc_length / lin_v
        if radius < 1000:
            ang_v = lin_v / radius
        else:
            ang_v = 0.0
        return lin_v, ang_v, time

    # Might want to have a "stupid" odometry of just header,x,y,yaw
    def odom_callback(self, msg):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        self.odom_yaw = msg.pose.pose.orientation


def main(args=None):
    rclpy.init(args=args)

    pursuit_action_server = PursuitActionServer()

    rclpy.spin(pursuit_action_server)


if __name__ == "__main__":
    main()
