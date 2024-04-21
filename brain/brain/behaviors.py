"""
This file defines the behaviors for the robot.

Used in `tree.py`.
"""

import py_trees as pt
import py_trees_ros as ptr
import rclpy.action
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import JointState
from robp_interfaces.action import Approach, Finetune, Arm
import rclpy
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from typing import TypedDict

# I know I should probably not use global variables, but not sure how to pass data
# between behaviors, so here are every behavior's global variables

# global variables
# current_object = '9'  # object id, or <aruco_id> for aruco markers


class TemplateBehaviour(pt.behaviour.Behaviour, Node):
    def __init__(self, name='TemplateBehaviour'):
        pt.behaviour.Behaviour.__init__(self, name=name)
        Node.__init__(self, node_name=name)

        self.blackboard = self.attach_blackboard_client(name)

    def register_bb(self, key, read_access=True, write_access=True):
        self.blackboard.register_key(
            key=key, access=pt.common.Access.READ if read_access else pt.common.Access.WRITE)
        self.blackboard.register_key(
            key=key, access=pt.common.Access.WRITE if write_access else pt.common.Access.READ)


class Initializer(pt.composites.Sequence):
    def __init__(self, name="Initializer"):
        super(Initializer, self).__init__(name=name, memory=True)

        self.add_children([ArmToHome()])


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
            # GetObjectPositionBehavior(),
            # PlanToObjectBehavior(),
            # ApproachObjectBehavior(),
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
            ApproachObjectBehavior(),
            FineTuneBoxPositionBehavior(),
            PlaceBehavior(),
        ])


class ArmToHome(TemplateBehaviour):
    def __init__(self, name="ArmToHome"):
        super().__init__(name=name)
        self.publisher_ = self.create_publisher(
            Int16MultiArray, '/multi_servo_cmd_sub', 10)
        self.subscriber_ = self.create_subscription(
            JointState, '/servo_pos_publisher', self.arm_pos_callback, 10)

        self.register_bb('current_target_object',
                         read_access=True, write_access=True)

        self.current_joint_pos = [12000, 12000, 12000, 12000, 12000, 12000]
        self.position_reached = False

        self.HOME_POSITION = [12000 for i in range(12)]
        for i in range(6):
            self.HOME_POSITION[i+6] = 800
        self.HOME_POSITION[0] = 0
        self.HOME_POSITION[1] = 12000
        self.HOME_POSITION[2] = 2000
        self.HOME_POSITION[3] = 16000
        self.HOME_POSITION[4] = 8000

    def initialise(self):
        self.publisher_.publish(Int16MultiArray(data=self.HOME_POSITION))
        self.blackboard.set('current_target_object', '1')

    def arm_pos_callback(self, msg: JointState):
        self.current_joint_pos = msg.position
        diff = 0
        for i in range(6):
            diff += abs(self.current_joint_pos[i] - self.HOME_POSITION[i])
        self.position_reached = diff < 2000

    def update(self):
        return pt.common.Status.SUCCESS


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


class ApproachObjectBehavior(TemplateBehaviour):
    """
    use realsense to approach the object
    """

    def __init__(self, name="ApproachObjectBehavior"):
        super().__init__(name=name)

        self.action_client = ActionClient(self, Approach, 'approach')
        self.future = None

        self.initialised = False

        self.change_camera_mode_pub = self.create_publisher(
            String, '/detection_ml/change_mode', 10)

        self.register_bb('current_target_object',
                         read_access=True, write_access=True)

    def initialise(self) -> None:
        super().initialise()
        self.change_camera_mode_pub.publish(String(data='front-camera'))

    def update(self):
        # global current_object

        rclpy.spin_once(self, timeout_sec=0.01)
        if not self.initialised or self.future is None:
            goal_msg = Approach.Goal()
            # goal_msg.target = current_object
            goal_msg.target = self.blackboard.current_target_object
            if not self.action_client.wait_for_server(timeout_sec=0.01):
                return pt.common.Status.RUNNING

            self.future = self.action_client.send_goal_async(goal_msg)

            self.initialised = True
            return pt.common.Status.RUNNING
        else:
            if not self.future.done():
                return pt.common.Status.RUNNING

            if self.future.result().status == GoalStatus.STATUS_EXECUTING:
                return pt.common.Status.RUNNING

            if self.future.result().status == GoalStatus.STATUS_SUCCEEDED:
                self.initialised = False
                self.future = None
                return pt.common.Status.SUCCESS
            self.initialised = False
            self.future = None
            return pt.common.Status.FAILURE


class FineTuneObjectPositionBehavior(TemplateBehaviour):
    """
    fine tune the object position with real sense detection
    """

    def __init__(self, name="FineTuneObjectPositionBehavior"):
        # pt.behaviour.Behaviour.__init__(self, name=name)
        # Node.__init__(self, node_name=name)
        super().__init__(name=name)

        # self.initialised = False
        self.action_client = ActionClient(self, Finetune, 'finetune')
        # self.future = None
        self.state = pt.common.Status.RUNNING
        self.camera_mode_pub = self.create_publisher(
            String, '/detection_ml/change_mode', 10)

        self.register_bb('current_target_object',
                         read_access=True, write_access=True)

        self.register_bb('pick_pos', read_access=True, write_access=True)

    def initialise(self) -> None:
        super().initialise()
        self.camera_mode_pub.publish(String(data='arm-camera'))

        self.action_client.wait_for_server()

        goal_msg = Finetune.Goal()
        goal_msg.object_id = self.blackboard.current_target_object
        self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.state = pt.common.Status.FAILURE

            self.send_goal_future = None
            self.get_result_future = None
            return

        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.blackboard.pick_pos = {
                'x': result.position[0],
                'y': result.position[1],
                'angle': result.angle
            }
            self.state = pt.common.Status.SUCCESS

        else:
            self.state = pt.common.Status.FAILURE

        self.send_goal_future = None
        self.get_result_future = None

    def update(self):
        rclpy.spin_once(self, timeout_sec=0.01)
        if self.state == pt.common.Status.FAILURE:
            self.state = pt.common.Status.RUNNING
            return pt.common.Status.FAILURE
        else:
            return self.state
        # global current_object


class PickObjectBehavior(TemplateBehaviour):
    """
    pick up the object with the arm
    """

    def __init__(self, name="PickObjectBehavior"):
        super().__init__(name=name)

        self.register_bb('pick_pos', read_access=True, write_access=True)

        self.action_client = ActionClient(self, Arm, 'arm')

        self.state = pt.common.Status.RUNNING

    def initialise(self) -> None:
        super().initialise()
        goal_msg = Arm.Goal()
        goal_msg.command = "pick"
        goal_msg.position = [float(self.blackboard.pick_pos['x']),
                             float(self.blackboard.pick_pos['y']) + 0.20]  # offset from arm_base to gripper
        goal_msg.angle = -self.blackboard.pick_pos['angle']

        self.action_client.wait_for_server()

        self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.state = pt.common.Status.FAILURE
            self.send_goal_future = None
            self.get_result_future = None
            return

        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.state = pt.common.Status.SUCCESS
        else:
            self.state = pt.common.Status.FAILURE

        self.send_goal_future = None
        self.get_result_future = None

    def update(self):
        rclpy.spin_once(self, timeout_sec=0.01)
        return self.state


class GetBoxPositionBehavior(pt.behaviour.Behaviour):
    """
    get the position of the box
    """

    def __init__(self, name="GetBoxPositionBehavior"):
        super(GetBoxPositionBehavior, self).__init__(name=name)

    def update(self):
        # place holder for the get box behavior
        return pt.common.Status.RUNNING


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
        pt.behaviour.Behaviour.__init__(self, name=name)
        Node.__init__(self, node_name=name)

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


class PickPosDict(TypedDict):
    # x and y in arm camera frame
    x: float
    y: float
    # angle measured clockwise from camera vertical axis
    angle: float
