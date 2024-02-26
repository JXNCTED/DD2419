#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mapping_interfaces.srv import PathPlan
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped


class NavGoalMove(Node):
    def __init__(self):
        super().__init__('nav_goal_move')
        self.cli = self.create_client(PathPlan, 'path_plan')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.futures = []
        self.req = PathPlan.Request()
        self.current_pose = PoseStamped()
        self.goal_pose = PoseStamped()
        self.path_planned_pub = self.create_publisher(
            Path, '/path_planned', 10)
        self.target_nav_sub_ = self.create_subscription(
            PoseStamped, '/goal_pose', self.target_nav_callback, 10)
        self.odom_sub_ = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

    def send_path_plan_request(self):
        self.get_logger().info('send_path_plan_request...')
        self.req.current_pose = self.current_pose
        self.req.goal_pose = self.goal_pose
        self.futures.append(self.cli.call_async(self.req))

    def target_nav_callback(self, msg: PoseStamped):
        self.get_logger().info('target_nav_callback...')
        self.goal_pose = msg
        self.send_path_plan_request()

    def odom_callback(self, msg: Odometry):
        self.current_pose.pose = msg.pose.pose
        self.current_pose.header = msg.header

    def loop(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            for future in self.futures:
                if future.done():
                    try:
                        response = future.result()
                        self.path_planned_pub.publish(response.path)
                        self.get_logger().info('Path planned published...')
                        self.futures.remove(future)
                    except Exception as e:
                        self.get_logger().warn('Service call failed %r' % (e,))


def main():
    rclpy.init()
    node = NavGoalMove()
    node.loop()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
