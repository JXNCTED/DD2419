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
        super(Initializer, self).__init__(name=name)
        timeout = pt.decorators.Timeout(
            name="Timeout", duration=5, children=[ArmToHome()])
        self.add_children([timeout])


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


# Publishes message to move arm into detect mode.
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


class OpenCloseGripper(pt.behaviour.Behaviour, Node):
    def __init__(self, name="CloseGripperObjectBehavior", action="open"):
        # super(GraspObjectBehavior, self).__init__(name=name)
        pt.behaviour.Behaviour.__init__(self, name=name)
        Node.__init__(self, node_name=name)
        self.publisher_ = self.create_publisher(String, '/arm_conf', 10)
        self.action = action
        # pubclisher

    def update(self):
        # place holder for the grasp object behavior
        self.publisher_.publish(String(data=self.action))
        return pt.common.Status.SUCCESS
  # This is the new class based on the code from above?


class MoveArmIntoDetect(pt.behaviour.Behaviour, Node):
    def __init__(self, name="GraspObjectBehavior"):
        # super(GraspObjectBehavior, self).__init__(name=name)
        pt.behaviour.Behaviour.__init__(self, name=name)
        Node.__init__(self, node_name=name)
        self.publisher_ = self.create_publisher(String, '/arm_conf', 10)
        # self.publisherActivateDetect = self.create_publisher(
        # Bool, '/can_detect', 10)
        # pubclisher

    def update(self):
        # place holder for the grasp object behavior
        self.publisher_.publish(String(data="detect"))
        # self.publisherActivateDetect.publish(Bool(date=True))
        return pt.common.Status.SUCCESS


# Moving arm into neutral position, can possibly change the other class to accomate different positions. Have to think..
class MoveArmIntoNeutral(pt.behaviour.Behaviour, Node):
    def __init__(self, name="MoveArmNeutral"):
        # super(GraspObjectBehavior, self).__init__(name=name)
        pt.behaviour.Behaviour.__init__(self, name=name)
        Node.__init__(self, node_name=name)
        self.publisher_ = self.create_publisher(String, '/arm_conf', 10)

    def update(self):
        # place holder for the grasp object behavior
        self.publisher_.publish(String(data="neutral"))
        return pt.common.Status.SUCCESS


class MoveArmIntoPickUp(pt.behaviour.Behaviour, Node):
    def __init__(self, name="MoveArmPickup"):
        # super(GraspObjectBehavior, self).__init__(name=name)
        pt.behaviour.Behaviour.__init__(self, name=name)
        Node.__init__(self, node_name=name)
        self.publisher_ = self.create_publisher(String, '/arm_conf', 10)

    def update(self):
        # place holder for the grasp object behavior
        self.publisher_.publish(String(data="pickup"))
        return pt.common.Status.SUCCESS

   # Basic functionality from MS2 for the arm in the behavior tree.
   # Sequence-->
       # 1. Listen to topic if object is within 25cm, if so. return SUCCESS.
       # 2. PUBLISH to topic /arm_conf to put arm into detect, .. also PUBLISH to detect topic to allow arm camera to detect (With appropriate filter applied), return SUCCESS.
       # 3. Using the node to detect and move. If object not in the middle, return RUNNING and keep adjusting robot position. If object is centered, stop moving and return SUCCESS.

       # SEQUENCE PICK UP -->
        # 4. PUBLISH to /arm_conf to PICK UP. Return SUCCESS
        # 5. Node that returns SUCCESS after some ticks.
        # 6. PUBLISH to /arm_conf to NEUTRAL. Return SUCCESS.
        # SELECTOR FOR CHECKING IF PICKED UP?


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
