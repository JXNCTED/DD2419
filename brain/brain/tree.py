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

        # Checking to see if object is within 25cm
        getDist = ptr.subscribers.CheckData(
            name="getDist?",
            topic_name="/dist_bool",
            topic_type=Bool,
            variable_name="data",
            qos_profile=ptr.utilities.qos_profile_unlatched(),
            expected_value=True,
        )

        # Checking if object is in the middle of the frame.
        getMiddlePos = ptr.subscribers.CheckData(
            name="isInMiddleOfFrame?",
            topic_name="/is_object_centered",
            topic_type=Bool,
            variable_name="data",
            qos_profile=ptr.utilities.qos_profile_unlatched(),
            expected_value=True,
        )

        # -------------------Counter to slow down the loop-------------------
        counter_fivethousand = pt.behaviours.TickCounter(
            name="TickCounter", duration=10, completion_status=pt.common.Status.SUCCESS)
        # -------------------Counter to slow down the loop-------------------

        self.pickupSequence = pt.composites.Sequence("MainTree", memory=True)

        self.pickupSequence.add_children([  # Pickup sequence
            MoveArmIntoPickUp(),  # moving into pickup
            counter_fivethousand,  # counter to slow down the loop
            CloseGripperObjectBehavior(),
            counter_fivethousand,  # counter to slow down the loop
            MoveArmIntoNeutral(),  # moving into neutral
        ])

        self.root.add_children([
            getDist,
            MoveArmIntoDetect(),  # Something off here?
            getMiddlePos,
            self.pickupSequence,
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
