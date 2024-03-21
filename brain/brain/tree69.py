import rclpy
from rclpy.node import Node
import py_trees
import py_trees_ros
import py_trees_ros_interfaces.msg as py_trees_msgs
from std_msgs.msg import Bool


class BehaviorTreeController(Node):
    def __init__(self):
        super().__init__('behavior_tree_controller')
        self.create_subscription(
            Bool,
            '/test_bool_sub',
            self.status_callback,
            10
        )
        self.publisher_ = self.create_publisher(
            Bool,
            '/test_bool_pub',
            10
        )

        self.tree = self.create_behavior_tree()
        self.blackboard = self.tree.blackboard

    def create_behavior_tree(self):
        idle = py_trees.behaviours.Running(name="Idle")
        walking = py_trees.behaviours.Running(name="Walking")
        running = py_trees.behaviours.Running(name="Running")

        idle_to_walking = py_trees.composites.Selector(
            "Idle to Walking", False)
        idle_to_walking.add_children([idle, walking])

        walking_to_running = py_trees.composites.Sequence(
            "Walking to Running", False)
        walking_to_running.add_children([walking, running])
        walking_to_running.add_child(
            py_trees.behaviours.TimedDecorator(
                name="WalkingTimeOut",
                child=py_trees.behaviours.Success(
                    name="WalkingToRunningSuccess"
                ),
                duration=5.0
            )
        )

        root = py_trees.composites.Sequence("Root")
        root.add_children([idle_to_walking, walking_to_running])

        tree = py_trees_ros.trees.BehaviourTree(root)

        return tree

    def status_callback(self, msg):
        # Logic to handle received status messages if needed
        pass

    def publish_control_message(self, command):
        msg = py_trees_msgs.BehaviorTreeStatus()
        # Populate message with appropriate data based on command
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    controller = BehaviorTreeController()
    try:
        rclpy.spin(controller)
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
