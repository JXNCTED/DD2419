import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from robp_interfaces.action import Pursuit
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class PursuitActionServer(Node):
    def __init__(self):
        super().__init__('pursuit_action_server')

        self.odom_x = 0
        self.odom_y = 0
        self.odom_yaw = 0

        self.action_server = ActionServer(
            self,
            Pursuit,
            'pursuit',
            self.execute_callback
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

    # Pseudo code sort of
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Pursuit.Feedback()
        Twist().linear.
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
        x = self.odom.x
        y = self.odom.y
        yaw = self.odom.yaw
        dx = target.x - self.odom.x
        dy = target.y - self.odom.y

    # Might want to have a "stupid" odometry of just header,x,y,yaw
    def odom_callback(self, msg):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        self.odom_yaw = msg.pose.pose.orientation
        Odometry().pose.pose.position


def main(args=None):
    rclpy.init(args=args)

    pursuit_action_server = PursuitActionServer()

    rclpy.spin(pursuit_action_server)


if __name__ == '__main__':
    main()
