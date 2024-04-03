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
        ])

        # Checking to see if object is within 25cm
        # getDist = ptr.subscribers.CheckData(
        #     name="getDist?",
        #     topic_name="/dist_bool",
        #     topic_type=Bool,
        #     variable_name="data",
        #     qos_profile=ptr.utilities.qos_profile_unlatched(),
        #     expected_value=True,
        # )

        # Checking if object is in the middle of the frame.
        # getMiddlePos = ptr.subscribers.CheckData(
        #     name="isInMiddleOfFrame?",
        #     topic_name="/is_object_centered",
        #     topic_type=Bool,
        #     variable_name="data",
        #     qos_profile=ptr.utilities.qos_profile_unlatched(),
        #     expected_value=True,
        # )

        # -------------------Counter to slow down the loop-------------------
        # c1 = pt.behaviours.TickCounter(
        #     name="TickCounter", duration=5, completion_status=pt.common.Status.SUCCESS)
        # c2 = pt.behaviours.TickCounter(
        #     name="TickCounter", duration=5, completion_status=pt.common.Status.SUCCESS)
        # c3 = pt.behaviours.TickCounter(
        #     name="TickCounter", duration=5, completion_status=pt.common.Status.SUCCESS)
        # c4 = pt.behaviours.TickCounter(
        #     name="TickCounter", duration=5, completion_status=pt.common.Status.SUCCESS)
        # c5 = pt.behaviours.TickCounter(
        #     name="TickCounter", duration=5, completion_status=pt.common.Status.SUCCESS)
        # c6 = pt.behaviours.TickCounter(
        #     name="TickCounter", duration=5, completion_status=pt.common.Status.SUCCESS)
        # -------------------Counter to slow down the loop-------------------

        # self.pickupSequence = pt.composites.Sequence(
        #     "PickUpSequence", memory=True)

        # self.pickupSequence.add_children([  # Pickup sequence
        #     MoveArmIntoPickUp(),  # moving into pickup
        #     c1,  # counter to slow down the loop
        #     OpenCloseGripper(action="close"),
        #     c2,  # counter to slow down the loop
        #     MoveArmIntoNeutral(),  # moving into neutral
        # ])

        # self.placeSequence = pt.composites.Sequence(
        #     "PlaceSequence", memory=True)
        # self.placeSequence.add_children([
        #     MoveArmIntoDetect(),
        #     c3,
        #     OpenCloseGripper(action="open"),
        #     c4,
        # ])  # Place sequence

        # self.root.add_children([
        #     getDist,
        #     MoveArmIntoDetect(),  # Something off here?
        #     getMiddlePos,
        #     self.pickupSequence,
        #     c5,
        #     self.placeSequence,
        #     MoveArmIntoNeutral(),
        #     c6,
        #     ExploreBehavior(),
        #     PickAndPlaceSelector(),
        # ])
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
