import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from robp_interfaces.action import Pursuit


class PursuitActionServer(Node):
    def __init__(self):
        super().__init__('pursuit_action_server')
        self.action_server = ActionServer(
            self,
            Pursuit,
            'pursuit',
            self.execute_callback
        )

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        result = Pursuit.Result()
        return result


def main(args=None):
    rclpy.init(args=args)

    pursuit_action_server = PursuitActionServer()

    rclpy.spin(pursuit_action_server)


if __name__ == '__main__':
    main()
