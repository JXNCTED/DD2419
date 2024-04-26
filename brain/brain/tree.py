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
        self.root.add_children([
            Initializer(),
            Exploration(),
            PPPP(),
        ])

        super(BehaviorTree, self).__init__(
            root=self.root, unicode_tree_debug=unicode_tree_debug)

        self.setup(timeout=15.0)


def main(argv=None):
    rclpy.init(args=argv)
    bt = BehaviorTree(unicode_tree_debug=True)
    pt.display.render_dot_tree(bt.root, with_blackboard_variables=True)

    bt.tick_tock(100)

    try:
        rclpy.spin(bt.node)
    except KeyboardInterrupt:
        pass

    bt.shutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
