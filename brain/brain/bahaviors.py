"""
This file defines the behaviors for the robot.

Used in `tree.py`.
"""

import py_trees as pt
import py_trees_ros as ptr


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
        return pt.common.Status.RUNNING


class ApproachObjectBehavior(pt.behaviour.Behaviour):
    def __init__(self, name="ApproachObjectBehavior"):
        super(ApproachObjectBehavior, self).__init__(name=name)

    def update(self):
        # place holder for the approach object behavior
        return pt.common.Status.RUNNING


class GraspObjectBehavior(pt.behaviour.Behaviour):
    def __init__(self, name="GraspObjectBehavior"):
        super(GraspObjectBehavior, self).__init__(name=name)

    def update(self):
        # place holder for the grasp object behavior
        return pt.common.Status.RUNNING


class PlaceObjectBehavior(pt.behaviour.Behaviour):
    def __init__(self, name="PlaceObjectBehavior"):
        super(PlaceObjectBehavior, self).__init__(name=name)

    def update(self):
        # place holder for the place object behavior
        return pt.common.Status.RUNNING


class GetBoxPositionBehavior(pt.behaviour.Behaviour):
    def __init__(self, name="GetBoxPositionBehavior"):
        super(GetBoxPositionBehavior, self).__init__(name=name)

    def update(self):
        # place holder for the get box position behavior
        return pt.common.Status.RUNNING


class ApproachBoxBehavior(pt.behaviour.Behaviour):
    def __init__(self, name="ApproachBoxBehavior"):
        super(ApproachBoxBehavior, self).__init__(name=name)

    def update(self):
        # place holder for the approach box behavior
        return pt.common.Status.RUNNING


class PlaceObjectBehavior(pt.behaviour.Behaviour):
    def __init__(self, name="PlaceObjectBehavior"):
        super(PlaceObjectBehavior, self).__init__(name=name)

    def update(self):
        # place holder for the place box behavior
        return pt.common.Status.RUNNING


class PickAndPlaceSelector(pt.composites.Selector):
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
