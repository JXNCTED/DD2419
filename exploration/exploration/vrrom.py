import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from robp_interfaces.action import NavGoal


class Vrrom(Node):
    def __init__(self):
        super().__init__('vrrom')
        self.ngm_action_client = ActionClient(self, NavGoal, "nav_goal_move")
        self.ngm_action_client.wait_for_server()
        self.target_nav_sub_ = self.create_subscription(
            PoseStamped, '/goal_pose', self.target_nav_callback, 10)

    def target_nav_callback(self, msg: PoseStamped):
        navGoal = NavGoal.Goal()
        navGoal.goal_pose = msg
        future = self.ngm_action_client.send_goal_async(navGoal)
    #     future.add_done_callback(self.ngm_goal_handle_callback)

    # def ngm_goal_handle_callback(self, future):
    #     goal_handle = future.result()


def main():
    rclpy.init()

    vrrom = Vrrom()
    try:
        rclpy.spin(vrrom)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == "__main__":
    main()
