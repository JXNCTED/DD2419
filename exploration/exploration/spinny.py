from typing import Tuple
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from rclpy.action import ActionClient, ActionServer
from geometry_msgs.msg import PoseStamped, Twist, PointStamped
from robp_interfaces.action import NavGoal, Explore
from nav_msgs.msg import OccupancyGrid
import numpy as np
from rclpy.callback_groups import ReentrantCallbackGroup
from detection_interfaces.msg import BoxList, StuffList, Stuff, Box
from mapping_interfaces.srv._get_frontier import GetFrontier
from threading import Condition
from rclpy.executors import MultiThreadedExecutor

super_category_to_box_id = {
    "none": 0,
    "cube": 1,
    "sphere": 2,
    "animal": 3,
}


class SpinnyExploration(Node):
    def __init__(self):
        super().__init__("spinny_exploration")

        # self.exploration_pose = PoseStamped()
        # self.unreachable_points = np.zeros((1, 2)) - 1000

        # self.occupancy_grid = OccupancyGrid()

        self.stuff_set = set()
        self.box_set = set()

        self._cb_group = ReentrantCallbackGroup()

        self.point = None

        self._action_client_condition = Condition()

        # self.run_sub = self.create_subscription(
        #     Bool,
        #     "exploration/run",
        #     self.run_exploration,
        #     10,
        #     callback_group=self._cb_group,
        # )

        self.nav_action_client = ActionClient(self, NavGoal, "navigator")

        self._action_server = ActionServer(
            self,
            Explore,
            "exploration",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._cb_group
        )

        self.found_correspondence = False
        self.done_exploring = False

        # self.debug_run_once = False

        self.rate = self.create_rate(100)  # 100 Hz

        self.get_frontier_client = self.create_client(
            GetFrontier, "/get_frontier")
        self.get_frontier_future = None

        # self.occupancy_grid_sub = self.create_subscription(
        #     OccupancyGrid,
        #     "/occupancy",
        #     self.update_occupancy_grid,
        #     10,
        #     callback_group=self._cb_group,
        # )

        self.twist_pub = self.create_publisher(
            Twist, "/motor_controller/twist", 10
        )

        self.frontier_point_pub = self.create_publisher(
            PointStamped,
            "/frontier_point",
            10
        )

        self.spin_rate = self.create_rate(10)

        self._stuff_sub = self.create_subscription(
            StuffList,
            '/category_eval/stuff_list',
            self.stuff_callback,
            10,
            callback_group=self._cb_group
        )

        self._box_sub = self.create_subscription(
            BoxList,
            '/box_list',
            self.box_callback,
            10,
            callback_group=self._cb_group
        )

        self.point = None

    def goal_callback(self, goal_request):
        self.get_logger().info("Goal request received")
        return rclpy.action.GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Goal cancelled")
        return rclpy.action.CancelResponse.ACCEPT

    def execute_callback(self, goal_handle) -> Explore.Result:
        self.get_logger().info("Goal executing")
        for _ in range(50):
            self.rate.sleep()
        while not self.found_correspondence and not self.done_exploring:
            get_frontier_req = GetFrontier.Request()
            self.get_frontier_client.wait_for_service()
            self.get_frontier_future = self.get_frontier_client.call_async(
                get_frontier_req)
            self.get_logger().info("Get frontier request sent")

            self.get_frontier_future.add_done_callback(
                self.get_frontier_done_callback)

            with self._action_client_condition:
                self.get_logger().info("Waiting for action client")
                self._action_client_condition.wait()

            self.rate.sleep()

        ret = Explore.Result()

        # no frontier point found, done exploring and success
        ret.success = self.done_exploring
        goal_handle.succeed()

        return ret

    # def run_exploration(self, msg: Bool):
    #     # when we can run
    #     if msg.data == True:

    #         self.get_logger().info("Exploration started")
    #         # navGoal = NavGoal.Goal()

    #         get_frontier_req = GetFrontier.Request()
    #         self.get_frontier_client.wait_for_service()
    #         self.get_frontier_future = self.get_frontier_client.call_async(
    #             get_frontier_req)
    #         self.get_logger().info("Get frontier request sent")

    #         self.get_frontier_future.add_done_callback(
    #             self.get_frontier_done_callback)

    #         # see if we have an object and a corresponding box
    #         # if len(self.stuff_set.intersection(self.box_set)) > 0:
    #         #     self.get_logger().info("Exploration finished")
    #         #     return

    #         # explore
    #         # self.select_next_exploration_pose()
    #         # self.get_logger().info("Exploration point selected")
    #         # # see if we have an object and a corresponding box
    #         # if len(self.stuff_set.intersection(self.box_set)) > 0:
    #         #     self.get_logger().info("Exploration finished")
    #         #     return

    #         # # set the goal pose and send it
    #         # navGoal.goal_pose = self.exploration_pose
    #         # self.get_logger().info("Exploration pose set")
    #         # print(navGoal.goal_pose)

    #         # self.ngm_action_client.wait_for_server()
    #         # future = self.ngm_action_client.send_goal_async(navGoal)
    #         # self.get_logger().info("Exploration pose sent")

    #         # future.add_done_callback(self.nav_goal_callback)
    #     # when we cannot run
    #     if msg.data == False:
    #         self.ngm_action_client.cancel_goal_async()
    #         self.get_logger().info("Exploration cancelled")

    def get_frontier_done_callback(self, future):
        response = future.result()
        if response.success:
            # a hack to spin the robot

            self.point = PointStamped()
            self.point.point = response.p
            self.point.header.frame_id = "map"
            self.point.header.stamp = self.get_clock().now().to_msg()
            self.frontier_point_pub.publish(self.point)

            twist = Twist()
            # for _ in range(200):
            #     twist.angular.z = 0.15
            #     self.twist_pub.publish(twist)
            #     self.spin_rate.sleep()
            # for _ in range(25):
            #     twist.angular.z = 0.0
            #     self.twist_pub.publish(twist)
            #     self.spin_rate.sleep()

            for _ in range(10):
                for _ in range(20):
                    twist.angular.z = 0.15
                    self.twist_pub.publish(twist)
                    self.spin_rate.sleep()
                for _ in range(20):
                    twist.angular.z = 0.0
                    self.twist_pub.publish(twist)
                    self.spin_rate.sleep()
            
            for _ in range(25):
                twist.angular.z = 0.0
                self.twist_pub.publish(twist)
                self.spin_rate.sleep()
            self.get_logger().info("Get frontier response received")
            # self.get_logger().info(f"{response.p.x}, {response.p.y}")

            nav_goal = NavGoal.Goal()
            nav_goal.goal_pose.pose.position = response.p
            nav_goal.goal_pose.header.frame_id = "map"
            nav_goal.goal_pose.header.stamp = self.get_clock().now().to_msg()

            self.nav_action_client.wait_for_server()

            nav_goal_future = self.nav_action_client.send_goal_async(nav_goal)
            nav_goal_future.add_done_callback(self.nav_goal_callback)
            self.get_logger().info("Exploration point selected")
        else:
            self.point = None
            self.done_exploring = True
            self.get_logger().info("Get frontier response failed")
            with self._action_client_condition:
                self._action_client_condition.notify_all()

        self.get_logger().info("Get frontier response received")

    def nav_goal_callback(self, future):
        # Here we get the goal_handle that checks if it has been accepted or not and other status updates.
        goal_handle = future.result()
        self.get_logger().info("Goal handle received")
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            with self._action_client_condition:
                self._action_client_condition.notify_all()
            return

        self.get_logger().info("Goal accepted")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_goal_result)
        # result_future.add(self.nav_goal_result)

    def nav_goal_result(self, future):
        self.get_logger().info("Result future received")
        # if future.cancelled():
        #     return
        # Nav goal has finished
        # result = future.result().result.done
        if not future.cancelled():
            result = future.result().result.done
            self.get_logger().info(f"Goal result: {result}")

        # notify all no matter what result
        with self._action_client_condition:
            self._action_client_condition.notify_all()
        # if result:
        #     self.get_logger().info("A point has been explored")
        # else:
        #     self.get_logger().info("Could not be explored")

    # def select_next_exploration_pose(self) -> bool:
    #     # a hack to spin the robot
    #     twist = Twist()
    #     for _ in range(100):
    #         twist.angular.z = 0.3
    #         self.twist_pub.publish(twist)
    #         self.spin_rate.sleep()
    #     for _ in range(25):
    #         twist.angular.z = 0.0
    #         self.twist_pub.publish(twist)
    #         self.spin_rate.sleep()

    #     # create the grid
    #     ori_x = self.occupancy_grid.info.origin.position.x
    #     ori_y = self.occupancy_grid.info.origin.position.y
    #     res = self.occupancy_grid.info.resolution

    #     self.get_logger().info(f"origin: {ori_x}, {ori_y}, {res}")

    #     if self.point == None:
    #         grid = np.array(self.occupancy_grid.data, dtype=np.int8).reshape(
    #             self.occupancy_grid.info.height, self.occupancy_grid.info.width
    #         )
    #         # make sure it is known but unoccupied
    #         unoccupied_points = np.argwhere(np.logical_and((grid >= 0), (grid < 80)))
    #         # pick a random point
    #         point = unoccupied_points[np.random.choice(np.arange(len(unoccupied_points)))]

    #         # transform from grid
    #         pose = PoseStamped()
    #         pose.pose.position.x = (point[1] + ori_x / res) * res
    #         pose.pose.position.y = (point[0] + ori_y / res) * res
    #     else:
    #         point = self.point

    #     self.exploration_pose = pose

    #     return False

    # def update_occupancy_grid(self, msg: OccupancyGrid):
    #     # update the occupancy grid
    #     self.occupancy_grid = msg

    def stuff_callback(self, msg: StuffList):
        stuff: Stuff
        if len(msg.data) == 0:
            return
        self.stuff_set.clear()
        for stuff in msg.data:
            if stuff is not None:
                self.stuff_set.add(
                    super_category_to_box_id[stuff.super_category])

        self.found_correspondence = len(
            self.stuff_set.intersection(self.box_set)) > 0

    def box_callback(self, msg: BoxList):
        if len(msg.boxes) == 0:
            return
        box: Box
        self.box_set.clear()
        for box in msg.boxes:
            if box is not None:
                self.box_set.add(box.aruco_id)


def main():
    rclpy.init()
    executor = rclpy.executors.MultiThreadedExecutor(8)
    node = SpinnyExploration()
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == "__main__":
    main()
