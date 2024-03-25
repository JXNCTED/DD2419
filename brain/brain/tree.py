#!/usr/bin/env python
"""
This is the main behavior tree for the robot. 

For behaviors, see `behaviors.py`.
"""
import py_trees as pt
import py_trees_ros as ptr
import rclpy
from std_msgs.msg import Bool, String
from .behaviors import *


class BehaviorTree(ptr.trees.BehaviourTree):
    def __init__(self, unicode_tree_debug=False):
        self.root = pt.composites.Sequence("MainTree", memory=True)

        getDist = ptr.subscribers.CheckData(
            name="getDist?",
            topic_name="/dist_bool",
            topic_type=Bool,
            variable_name="data",
            qos_profile=ptr.utilities.qos_profile_unlatched(),
            expected_value=False,
        )

        self.root.add_children([
            getDist,
            ExploreBehavior(),
            PickAndPlaceSelector(),
        ])
        super(BehaviorTree, self).__init__(
            root=self.root, unicode_tree_debug=unicode_tree_debug)

        self.setup(timeout=15.0)


def main(argv=None):
    rclpy.init(args=argv)
    bt = BehaviorTree(unicode_tree_debug=True)
    pt.display.render_dot_tree(bt.root)

    bt.tick_tock(1000)

    try:
        rclpy.spin(bt.node)
    except KeyboardInterrupt:
        pass

    bt.shutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
