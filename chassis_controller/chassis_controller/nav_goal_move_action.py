#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mapping_interfaces.srv import PathPlan
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient, ActionServer, GoalResponse, CancelResponse
from geometry_msgs.msg import Point, Twist
from robp_interfaces.action import Pursuit
from rclpy.callback_groups import ReentrantCallbackGroup
from robp_interfaces.action import NavGoal


class NavGoalMove(Node):
    def __init__(self):
        super().__init__("nav_goal_move")
        self.cli = self.create_client(PathPlan, "path_plan")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        self.futures = []
        # request object
        self.req = PathPlan.Request()
        self.current_pose = PoseStamped()

        self._cb_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self,
            NavGoal,
            "nav_goal_move",
            execute_callback=self.execute_callback,
            callback_group=self._cb_group,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.path_planned_pub = self.create_publisher(
            Path, "/path_planned", 10)

        self.twist_pub = self.create_publisher(
            Twist, "/motor_controller/twist", 10
        )

        self.pursuit_client = ActionClient(self, Pursuit, "pursuit")
        self._loop_rate = self.create_rate(1, self.get_clock())
        
        # hack spin rate
        self.spin_rate = self.create_rate(100)

    def destroy(self):
        """Cleanup"""
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        self.get_logger().info("Accepting the goal request")
        return GoalResponse.ACCEPT

    def cancel_callback(self, cancel_request):
        """Accepts the cancel requests"""
        self.get_logger().info("Accepting cancel request")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.req.goal_pose = goal_handle.request.goal_pose
        self.impossible = False
        self.at_goal = False
        self.universal_goal_handle = goal_handle

        result = NavGoal.Result()
        result.done = False

        twist = Twist()

        while not self.impossible:
            self.stop_pursuit = False
            future_path = self.cli.call_async(self.req)
            future_path.add_done_callback(self.path_callback)
            for _ in range(10):
                self._loop_rate.sleep()
            if self.at_goal:
                result.done = True
                goal_handle.succeed()
                self.get_logger().info("at goal, doing 360")
                # a hack to spin at the endof the nav goal
                for _ in range(400):
                    twist.angular.z = 0.6
                    self.twist_pub.publish(twist)
                    self.spin_rate.sleep()
                for _ in range(100):
                    twist.angular.z = 0.0
                    self.twist_pub.publish(twist)
                    self.spin_rate.sleep()
                return result
                # break
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.done = False
                return result
                # break
            self.stop_pursuit = True
            self._loop_rate.sleep()
        result.done = False
        goal_handle.abort()
        return result

    def path_callback(self, future_path):
        response = future_path.result()
        if len(response.path.poses) == 0:
            self.get_logger().warn("Path planning failed")
            self.impossible = True
            return
        self.path_planned_pub.publish(response.path)
        self.get_logger().info("Path planned published...")
        waypoints = []
        for pose in response.path.poses:
            # TODO: transform wazpoints from map to odom frame
            waypoints.insert(
                0, Point(x=pose.pose.position.x, y=pose.pose.position.y))
        goal_msg = Pursuit.Goal()
        goal_msg.waypoints = waypoints

        if self.universal_goal_handle.is_cancel_requested:
            return

        self.pursuit_client.wait_for_server()
        send_pursuit = self.pursuit_client.send_goal_async(goal_msg)
        send_pursuit.add_done_callback(self.pursuit_goal_callback)

    def pursuit_goal_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Pursuit goal rejected")
            self.impossible = True
            return
        self.get_logger().info("Pursuit goal accepted")

        if self.universal_goal_handle.is_cancel_requested:
            # There is a future, I don't care. But perhaps the cancel_request should wait for this to cancel before canceling itself.
            goal_handle.cancel_goal_async()
            return

        pursuit_result = goal_handle.get_result_async()
        pursuit_result.add_done_callback(self.pursuit_result_callback)
        while rclpy.ok():
            if self.stop_pursuit:
                goal_handle.cancel_goal_async()
                self.stop_pursuit = False
                break
            if pursuit_result.done():
                break

    def pursuit_result_callback(self, future):
        pursuit_result = future.result().result
 
        if pursuit_result.success:
            self.get_logger().info("Pursuit succeeded")
            self.at_goal = True

        else:
            self.get_logger().info("Pursuit failed")
            self.impossible = True


def main():
    rclpy.init()

    executor = rclpy.executors.MultiThreadedExecutor(
        8
    )  # TODO check if 3 is the right number
    nav_goal_move = NavGoalMove()
    try:
        rclpy.spin(nav_goal_move, executor=executor)
    except KeyboardInterrupt:
        pass

    nav_goal_move.destroy()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
