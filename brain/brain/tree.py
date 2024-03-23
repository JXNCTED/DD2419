#!/usr/bin/env python

import py_trees as pt
import py_trees_ros as ptr
import rclpy


class SimpleBehavior(pt.behaviour.Behaviour):
    def __init__(self, name="SimpleBehavior"):
        super(SimpleBehavior, self).__init__(name=name)

    def update(self):
        print("SimpleBehavior: Running")
        return pt.common.Status.RUNNING


class SimpleBehavior2(pt.behaviour.Behaviour):
    def __init__(self, name="SimpleBehavior2"):
        super(SimpleBehavior2, self).__init__(name=name)

    def update(self):
        print("SimpleBehavior2: Running")
        return pt.common.Status.RUNNING


class BehaviorTree(ptr.trees.BehaviourTree):
    def __init__(self):
        self.root = pt.composites.Sequence("sequence", memory=True)
        self.root.add_children([SimpleBehavior(), SimpleBehavior2()])
        super(BehaviorTree, self).__init__(
            root=self.root, unicode_tree_debug=True)

        self.setup(timeout=15.0)


def main(argv=None):
    rclpy.init(args=argv)

    bt = BehaviorTree()
    bt.tick_tock(500)

    rclpy.spin(bt.node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
