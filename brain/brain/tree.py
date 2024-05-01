#!/usr/bin/env python
"""
This is the main behavior tree for the robot. 

For behaviors, see `behaviors.py`.
"""
import py_trees as pt
import py_trees.console as console
import py_trees_ros as ptr
import rclpy
from std_msgs.msg import Bool, String
from .behaviors import *
import functools


class BehaviorTree(ptr.trees.BehaviourTree):
    def __init__(self, unicode_tree_debug=False):
        self.root = pt.composites.Sequence("MainTree", memory=True)
        self.root.add_children([
            Initializer(),
            pt.decorators.FailureIsRunning(name="main sequence failure is runnning", child=MainSequence()),
        ])

        super(BehaviorTree, self).__init__(
            root=self.root, unicode_tree_debug=unicode_tree_debug)

        self.setup(timeout=15.0)

def post_tick_handler(snapshot_visitor, behaviour_tree):
    print("\n---------\n")
    print(pt.display.unicode_blackboard())
    print("\n---------\n")

def main(argv=None):
    rclpy.init(args=argv)
    bt = BehaviorTree(unicode_tree_debug=True)
    pt.display.render_dot_tree(bt.root, with_blackboard_variables=True)
    snapshot_visitor = pt.visitors.SnapshotVisitor()
    bt.add_post_tick_handler(functools.partial(post_tick_handler, snapshot_visitor))


    bt.tick_tock(100)

    try:
        rclpy.spin(bt.node)
    except KeyboardInterrupt:
        pass

    bt.shutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
