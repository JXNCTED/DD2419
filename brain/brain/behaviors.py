"""
This file defines the behaviors for the robot.

Used in `tree.py`.
"""

import py_trees as pt
import py_trees_ros as ptr
from rclpy.node import Node
from std_msgs.msg import String


class ExploreBehavior(pt.behaviour.Behaviour):
    def __init__(self, name="ExploreBehavior"):
        super(ExploreBehavior, self).__init__(name=name)

    def update(self):
        # place holder for the explore behavior
        return pt.common.Status.SUCCESS


class GetObjectPositionBehavior(pt.behaviour.Behaviour):
    def __init__(self, name="GetObjectPositionBehavior"):
        super(GetObjectPositionBehavior, self).__init__(name=name)

    def update(self):
        # place holder for the get object behavior
        return pt.common.Status.SUCCESS


class ApproachObjectBehavior(pt.behaviour.Behaviour):
    def __init__(self, name="ApproachObjectBehavior"):
        super(ApproachObjectBehavior, self).__init__(name=name)

    def update(self):
        # place holder for the approach object behavior
        return pt.common.Status.SUCCESS


class GraspObjectBehavior(pt.behaviour.Behaviour, Node):
    def __init__(self, name="GraspObjectBehavior"):
        # super(GraspObjectBehavior, self).__init__(name=name)
        pt.behaviour.Behaviour.__init__(self, name=name)
        Node.__init__(self, node_name=name)
        self.publisher_ = self.create_publisher(String, '/arm_conf', 10)
        # pubclisher

    def update(self):
        # place holder for the grasp object behavior
        self.publisher_.publish(String(data="detect"))
        return pt.common.Status.SUCCESS


class PlaceObjectBehavior(pt.behaviour.Behaviour):
    def __init__(self, name="PlaceObjectBehavior"):
        super(PlaceObjectBehavior, self).__init__(name=name)

    def update(self):
        # place holder for the place object behavior
        return pt.common.Status.SUCCESS


class GetBoxPositionBehavior(pt.behaviour.Behaviour):
    def __init__(self, name="GetBoxPositionBehavior"):
        super(GetBoxPositionBehavior, self).__init__(name=name)

    def update(self):
        # place holder for the get box position behavior
        return pt.common.Status.SUCCESS


class ApproachBoxBehavior(pt.behaviour.Behaviour):
    def __init__(self, name="ApproachBoxBehavior"):
        super(ApproachBoxBehavior, self).__init__(name=name)

    def update(self):
        # place holder for the approach box behavior
        return pt.common.Status.SUCCESS


class PlaceObjectBehavior(pt.behaviour.Behaviour):
    def __init__(self, name="PlaceObjectBehavior"):
        super(PlaceObjectBehavior, self).__init__(name=name)

    def update(self):
        # place holder for the place box behavior
        return pt.common.Status.SUCCESS


class PickAndPlaceSelector(pt.composites.Sequence):
    def __init__(self, name="PickAndPlaceBehavior"):
        super(PickAndPlaceSelector, self).__init__(name=name, memory=True)
        self.add_children([
            GetObjectPositionBehavior(),
            ApproachObjectBehavior(),
            GraspObjectBehavior(),
            GetBoxPositionBehavior(),
            ApproachBoxBehavior(),
            PlaceObjectBehavior()
        ])
