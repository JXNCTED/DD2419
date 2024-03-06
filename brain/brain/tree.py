import py_trees as pt
import py_trees_ros as ptr
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray

# https://medium.com/@nullbyte.in/behavior-trees-for-ros2-part-1-unlocking-advanced-robotic-decision-making-and-control-7856582fb812
# Need to create cpp file of behavior tree


class Counter(pt.behaviour.Behaviour):
    """
    Returns running for n ticks and success thereafter.
    """

    def __init__(self, n, name):
        # rclpy.loginfo("Initialising counter behaviour.")
        # counter
        self.i = 0
        self.n = n

        # become a behaviour
        super(Counter, self).__init__(name)

    def update(self):
        # increment i
        self.i += 1
        # rclpy.loginfo("Moving..............................................")

        # succeed after count is done
        return pt.common.Status.FAILURE if self.i <= self.n else pt.common.Status.SUCCESS


class BehaviourTree(Node):  # Inherit from rclpy.node.Node
    def __init__(self):
        super().__init__('BehaviourTree')
        self.arm_pub_ = self.create_publisher(
            Int16MultiArray, '/multi_servo_cmd_sub', 10)

        self.detect_angles = [12000 for i in range(12)]
        # set the speed of all the controls
        for i in range(6):
            self.detect_angles[i+6] = 1000
        # set conf
        self.detect_angles[2] = 2000
        self.detect_angles[3] = 18000
        self.detect_angles[4] = 10000
        # self.detect_angles[0] = self.open_gripper
        # Import servo values for detect phase for example.

        #  Move robot to detect phase.
        b0 = pt.composites.Selector(
            name="Move to detect phase",
            children=[Counter(10, "Moving to detect phase?"),
                      pt.behaviour(self.arm_pub_.publish, self.detect_angles)]

        )

        tree = pt.composites.Sequence(name="Main sequence", children=[b0])
        self.behaviour_tree = ptr.trees.BehaviourTree(
            tree, self)  # Create the BehaviourTree instance

    def loop(self):
        # Run the behaviour tree indefinitely
        self.behaviour_tree.setup(timeout=15)
        while rclpy.ok():
            self.behaviour_tree.tick_tock(1.0)
            rclpy.spin_once(self)


def main():
    rclpy.init()
    node = BehaviourTree()
    node.loop()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
