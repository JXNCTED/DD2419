"""
This file defines the behaviors for the robot.

Used in `tree.py`.
"""

import py_trees as pt
import py_trees_ros as ptr
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import JointState


class Initializer(pt.composites.Sequence):
    def __init__(self, name="Initializer"):
        super(Initializer, self).__init__(name=name, memory=True)
        timeout = pt.decorators.Timeout(
            name="Timeout", duration=2, child=ArmToHome())
        retry = pt.decorators.Retry(
            name="Retry", child=timeout, num_failures=3)
        failure_is_success = pt.decorators.FailureIsSuccess(
            name="FailureIsSuccess", child=retry)
        self.add_children([failure_is_success])


class Exploration(pt.composites.Sequence):
    def __init__(self, name="Exploration"):
        super(Exploration, self).__init__(name=name, memory=True)
        self.add_children([
            ExplorePointBehavior(),
            CheckExplorationCompletion(),
        ])


class Pick(pt.composites.Sequence):
    """
    for select object, go to object, fine tune object position, and pick up
    """

    def __init__(self, name="Pick"):
        super(Pick, self).__init__(name=name, memory=True)
        self.add_children([
            GetObjectPositionBehavior(),
            PlanToObjectBehavior(),
            FineTuneObjectPositionBehavior(),
            PickObjectBehavior(),
        ])


class Place(pt.composites.Sequence):
    """
    for get box position, plan to box, fine tune box position, and place
    """

    def __init__(self, name="Place"):
        super(Place, self).__init__(name=name, memory=True)
        self.add_children([
            GetBoxPositionBehavior(),
            PlanToBoxBehavior(),
            FineTuneBoxPositionBehavior(),
            PlaceBehavior(),
        ])


class ArmToHome(pt.behaviour.Behaviour, Node):
    def __init__(self, name="ArmToHome"):
        pt.behaviour.Behaviour.__init__(self, name=name)
        Node.__init__(self, node_name=name)
        self.publisher_ = self.create_publisher(
            Int16MultiArray, '/multi_servo_cmd_sub', 10)
        self.subscriber_ = self.create_subscription(
            JointState, '/servo_pos_publisher', self.arm_pos_callback, 10)

        self.current_joint_pos = [12000, 12000, 12000, 12000, 12000, 12000]
        self.position_reached = False

        self.HOME_POSITION = [12000 for i in range(12)]
        for i in range(6):
            self.HOME_POSITION[i+6] = 800
        self.HOME_POSITION[0] = 5000
        self.HOME_POSITION[1] = 12000
        self.HOME_POSITION[2] = 2000
        self.HOME_POSITION[3] = 18000
        self.HOME_POSITION[4] = 10000

    def initialise(self):
        self.publisher_.publish(Int16MultiArray(data=self.HOME_POSITION))

    def arm_pos_callback(self, msg: JointState):
        self.current_joint_pos = msg.position
        diff = 0
        for i in range(6):
            diff += abs(self.current_joint_pos[i] - self.HOME_POSITION[i])
        self.position_reached = diff < 2000

    def update(self):
        if self.position_reached:
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.RUNNING


class GetObjectPositionBehavior(pt.behaviour.Behaviour):
    """
    select one object from the object list and get the position of the object
    """

    def __init__(self, name="GetObjectPositionBehavior"):
        super(GetObjectPositionBehavior, self).__init__(name=name)
        # Initalize the subscriber

    def update(self):
        # place holder for the get object behavior
        # Get the information from the topic.
        return pt.common.Status.SUCCESS


class PlanToObjectBehavior(pt.behaviour.Behaviour):
    """
    use A star and pure pursuit to navigate to the object
    """

    def __init__(self, name="PlanToObjectBehavior"):
        super(PlanToObjectBehavior, self).__init__(name=name)

    def update(self):
        # place holder for the plan to object behavior
        return pt.common.Status.SUCCESS


class FineTuneObjectPositionBehavior(pt.behaviour.Behaviour):
    """
    fine tune the object position with real sense detection
    """

    def __init__(self, name="FineTuneObjectPositionBehavior"):
        super(FineTuneObjectPositionBehavior, self).__init__(name=name)

    def update(self):
        # place holder for the fine tune object position behavior
        return pt.common.Status.SUCCESS


class PickObjectBehavior(pt.behaviour.Behaviour):
    """
    pick up the object with the arm
    """

    def __init__(self, name="PickObjectBehavior"):
        super(PickObjectBehavior, self).__init__(name=name)

    def update(self):
        # place holder for the pick object behavior
        return pt.common.Status.SUCCESS


class GetBoxPositionBehavior(pt.behaviour.Behaviour):
    """
    get the position of the box
    """

    def __init__(self, name="GetBoxPositionBehavior"):
        super(GetBoxPositionBehavior, self).__init__(name=name)

    def update(self):
        # place holder for the get box behavior
        return pt.common.Status.SUCCESS


class PlanToBoxBehavior(pt.behaviour.Behaviour):
    """
    plan to the box with A star and pure pursuit
    """

    def __init__(self, name="PlanToBoxBehavior"):
        super(PlanToBoxBehavior, self).__init__(name=name)

    def update(self):
        # place holder for the plan to box behavior
        return pt.common.Status.SUCCESS


class FineTuneBoxPositionBehavior(pt.behaviour.Behaviour):
    """
    fine tune the box position with real sense detection
    """

    def __init__(self, name="FineTuneBoxPositionBehavior"):
        super(FineTuneBoxPositionBehavior, self).__init__(name=name)

    def update(self):
        # place holder for the fine tune box position behavior
        return pt.common.Status.SUCCESS


class PlaceBehavior(pt.behaviour.Behaviour, Node):
    """
    place the object in the box
    """

    def __init__(self, name="PlaceBehavior"):
        super(PlaceBehavior, self).__init__(name=name)
        # Send info to move arm into correct position
        # Pause
        # Send info to open gripper
        # Pause
        pt.behaviour.Behaviour.__init__(self, name=name)
        Node.__init__(self, node_name=name)
        self.publisher_ = self.create_publisher(
            Int16MultiArray, '/multi_servo_cmd_sub', 10)
        self.subscriber_ = self.create_subscription(
            JointState, '/servo_pos_publisher', self.arm_pos_callback, 10)

        self.current_joint_pos = [12000, 12000, 12000, 12000, 12000, 12000]
        self.position_reached = False
        # Use the callback here??
        self.PLACE_POSITION = [12000 for i in range(12)]
        for i in range(6):
            self.PLACE_POSITION[i+6] = 800

        # Replace these values with the correct place-arm-values. :)
        self.PLACE_POSITION[0] = 5000
        self.PLACE_POSITION[1] = 12000
        self.PLACE_POSITION[2] = 2000
        self.PLACE_POSITION[3] = 18000
        self.PLACE_POSITION[4] = 10000

    def update(self):
        # place holder for the place behavior

        return pt.common.Status.SUCCESS


class PlaceObjectBehavior(pt.behaviour.Behaviour):
    def __init__(self, name="PlaceObjectBehavior"):
        super(PlaceObjectBehavior, self).__init__(name=name)

    def update(self):
        # place holder for the place box behavior

        return pt.common.Status.SUCCESS


class CheckTaskCompletion(pt.behaviour.Behaviour):
    def __init__(self, name="CheckTaskCompletion"):
        super(CheckTaskCompletion, self).__init__(name=name)

    def update(self):
        # place holder for the check task completion behavior
        return pt.common.Status.RUNNING


class ExplorePointBehavior(pt.behaviour.Behaviour):
    def __init__(self, name="ExplorePointBehavior"):
        super(ExplorePointBehavior, self).__init__(name=name)

    def update(self):
        # place holder for the explore point behavior
        return pt.common.Status.SUCCESS


class CheckExplorationCompletion(pt.behaviour.Behaviour):
    def __init__(self, name="CheckExplorationCompletion"):
        super(CheckExplorationCompletion, self).__init__(name=name)

    def update(self):
        # place holder for the check exploration completion behavior
        return pt.common.Status.SUCCESS
