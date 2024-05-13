import rclpy
from geometry_msgs.msg import Point, Twist
from mapping_interfaces.srv import PathPlan
from nav_msgs.msg import Path
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from robp_interfaces.action import NavGoal, Pursuit
import threading

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor


class Navigator(Node):
    def __init__(self):
        super().__init__("navigator")
        self.is_moving = False

        self.cli = self.create_client(PathPlan, "path_plan")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")

        self._condition = threading.Condition()
        self._action_server = ActionServer(
            self,
            NavGoal,
            "navigator",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup()
        )

        self.path_planned_pub = self.create_publisher(
            Path, "/path_planned", 10)

        self.pursuit_client = ActionClient(self, Pursuit, "pursuit")

    def destroy(self):
        """Cleanup"""
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        if self.is_moving:
            self.get_logger().info("Rejecting the goal request")
            return GoalResponse.REJECT
        else:
            self.get_logger().info("Accepting the goal request")
            return GoalResponse.ACCEPT

    def cancel_callback(self, cancel_request):
        """Accepts the cancel requests"""
        self.get_logger().info("Accepting cancel request")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle: NavGoal):
        """Execute the pursuit goal"""
        self.is_moving = True

        req = PathPlan.Request()
        req.goal_pose = goal_handle.request.goal_pose
        future_path = self.cli.call_async(req)
        future_path.add_done_callback(self.path_callback)

        with self._condition:
            self.get_logger().info("get lock, wait for everything")
            self._condition.wait()
        ret = NavGoal.Result()

        self.get_logger().info("Goal reached")
        goal_handle.succeed()
        ret.done = True
        return ret

    def path_callback(self, future):
        path = future.result().path
        if len(path.poses) == 0:
            self.is_moving = False
            with self._condition:
                self._condition.notify_all()
            return

        self.get_logger().info("Path planned")
        self.path_planned_pub.publish(path)
        wazpoints = []
        for pose in path.poses:
            wazpoints.insert(
                0, Point(x=pose.pose.position.x, y=pose.pose.position.y))
        goal_msg = Pursuit.Goal()
        goal_msg.waypoints = wazpoints
        self.pursuit_client.wait_for_server()
        future_pursuit = self.pursuit_client.send_goal_async(
            goal_msg
        )  # , feedback_callback=self.feedback_callback)
        future_pursuit.add_done_callback(self.pursuit_callback)

    def pursuit_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Pursuit rejected")
            self.is_moving = False
            with self._condition:
                self._condition.notify_all()
            return
        self.get_logger().info("Pursuit accepted")
        res = goal_handle.get_result_async()
        res.add_done_callback(self.pursuit_result_callback)

    def pursuit_result_callback(self, future):
        self.is_moving = False
        with self._condition:
            self._condition.notify_all()


def main(args=None):
    rclpy.init(args=args)
    navigator = Navigator()
    executor = MultiThreadedExecutor(8)
    rclpy.spin(navigator, executor=executor)
    navigator.destroy()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
