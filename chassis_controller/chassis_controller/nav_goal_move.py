
import rclpy
from rclpy.node import Node


class NavGoalMove(Node):
    def __init__(self):
        super().__init__('nav_goal_move')


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
