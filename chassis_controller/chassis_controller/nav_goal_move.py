#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mapping_interfaces.msv import PathPlan
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped

class NavGoalMove(Node):
    def __init__(self):
        super().__init__('nav_goal_move')
        self.cli = self.create_client(PathPlan, 'path_plan')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = PathPlan.Request()
        self.current_pose = PoseStamped()
        self.target_pose = PoseStamped()
        self.path_planned_pub = self.create_publisher(Path, '/path_planned', 10)
        self.target_nav_sub_ = self.create_subscription(PoseStamped, '/move_base_simple/goal', self.target_nav_callback, 10)
        self.odom_sub_ = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
    
    def send_path_plan_request(self):
        self.req.current_pose = self.current_pose
        self.req.target_pose = self.target_pose
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        self.get_logger().info('sending request...')
        return self.future.result()
    
    def target_nav_callback(self, msg: PoseStamped):
        self.get_logger().info('target_nav_callback...')
        self.target_pose = msg
        response = self.send_path_plan_request()
        self.path_planned_pub.publish(response.path)
    
    def odom_callback(self, msg: Odometry):
        self.current_pose = msg.pose.pose

def main():
    rclpy.init()
    node = NavGoalMove()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
