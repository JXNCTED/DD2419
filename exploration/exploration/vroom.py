import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from robp_interfaces.action import NavGoal
from nav_msgs.msg import OccupancyGrid
import numpy as np
from rclpy.callback_groups import ReentrantCallbackGroup


class Exploration(Node):
    def __init__(self):
        super().__init__("exploration")

        self.exploration_pose = PoseStamped()
        self.unreachable_points = np.zeros((1, 2)) - 1000

        self.occupancy_grid = OccupancyGrid()
        self._cb_group = ReentrantCallbackGroup()

        self.run_sub = self.create_subscription(
            Bool,
            "exploration/run",
            self.run_exploration,
            10,
            callback_group=self._cb_group,
        )

        self.ngm_action_client = ActionClient(self, NavGoal, "nav_goal_move")

        self.occupancy_grid_sub = self.create_subscription(
            OccupancyGrid,
            "/occupancy",
            self.update_occupancy_grid,
            10,
            callback_group=self._cb_group,
        )

    def run_exploration(self, msg: Bool):
        # when we can run
        if msg.data == True:
            self.get_logger().info("Exploration started")
            navGoal = NavGoal.Goal()

            # see if we can explore
            done = self.select_next_exploration_pose()
            self.get_logger().info("Exploration point selected")
            if done == True:
                self.get_logger().info("Exploration finished")
                return

            # set the goal pose and send it
            navGoal.goal_pose = self.exploration_pose
            self.get_logger().info("Exploration pose set")
            print(navGoal.goal_pose)

            self.ngm_action_client.wait_for_server()
            future = self.ngm_action_client.send_goal_async(navGoal)
            self.get_logger().info("Exploration pose sent")

            future.add_done_callback(self.nav_goal_callback)
        # when we cannot run
        if msg.data == False:
            self.ngm_action_client.cancel_goal_async()
            self.get_logger().info("Exploration cancelled")

    def nav_goal_callback(self, future):
        # Here we get the goal_handle that checks if it has been accepted or not and other status updates.
        goal_handle = future.result()
        self.get_logger().info("Goal handle received")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_goal_result)

    def nav_goal_result(self, future):
        self.get_logger().info("Result future received")
        if future.cancelled():
            return
        # Nav goal has finished
        result = future.result().result.done
        if result:
            self.get_logger().info("A point has been explored")
        else:
            self.get_logger().info("Could not be explored")

    def select_next_exploration_pose(self) -> bool:
        # create the grid
        ori_x = self.occupancy_grid.info.origin.position.x
        ori_y = self.occupancy_grid.info.origin.position.y
        res = self.occupancy_grid.info.resolution

        grid = np.asarray(self.occupancy_grid.data).reshape(
            (self.occupancy_grid.info.width, self.occupancy_grid.info.height)
        )

        # unknown_indices = np.argwhere(grid == -1)

        # we subsample by subsam, naive/raw, no gauss
        ss_mult = 10  # how much we recude the grid
        # subsampled_grid = grid[::ss_mult, ::ss_mult].copy()
        subsampled_grid = np.any(grid.reshape(
            50, ss_mult, -1, ss_mult).swapaxes(1, 2).reshape(2500, -1) == -1, axis=1).reshape(50, 50)

        ss_shape  = (self.occupancy_grid.info.width // ss_mult, self.occupancy_grid.info.height // ss_mult)
        self.get_logger().info(f"Subsampled grid shape: {ss_shape}")
        subsampled_grid = np.any(grid.reshape(ss_shape[0], ss_mult, -1, ss_mult).swapaxes(1, 2).reshape(ss_shape[0] * ss_shape[1], -1) == -1, axis=1).reshape(ss_shape[0], ss_shape[1])


        with open("subsampled_grid.txt", "w") as f:
            for row in subsampled_grid:
                for cell in row:
                    # f.write(str(cell) + " ")
                    f.write(f"{1 if cell else 0} ")
                f.write("\n")
        unknown_indices = np.argwhere(subsampled_grid == -1)
        print(unknown_indices.shape)

        pose = PoseStamped()

        # check all the unknown points
        # for pt in unknown_indices:
        #     self.get_logger().info("Checking a point")
        #     # make sure this point is not in the unreachable list
        # pt = unknown_indices.
        # pick randmo pt
        pt = unknown_indices[np.random.randint(0, unknown_indices.shape[0])]
        print(pt)
        # pt = unknown_indices[0]
        if not np.any(np.all(self.unreachable_points == pt, axis=1)):
            self.get_logger().info("Point selected")
            # add to unreachable points and set the pose
            self.unreachable_points = np.append(
                self.unreachable_points, np.atleast_2d(pt), 0)
            pose.pose.position.x = (pt[1] * ss_mult * res) + ori_x
            pose.pose.position.y = (pt[0] * ss_mult * res) + ori_y

            self.exploration_pose = pose

            # we are not done - maybe more points to explore
            return False
        # we are done - no more points to explore
        return True

    def update_occupancy_grid(self, msg: OccupancyGrid):
        # count how many unknown points are there
        count = 0
        for idx, data in enumerate(self.occupancy_grid.data):
            if data == -1:
                count += 1
        # print(count)
        # update the occupancy grid
        self.occupancy_grid = msg


def main():
    rclpy.init()
    executor = rclpy.executors.MultiThreadedExecutor(2)
    node = Exploration()
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == "__main__":
    main()
