"""
This file defines the behaviors for the robot.

Used in `tree.py`.
"""

import py_trees as pt
from py_trees import decorators
from py_trees.common import Status
import py_trees_ros as ptr
import rclpy.action
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import JointState
from robp_interfaces.action import Approach, Finetune, Arm, Explore, Pursuit
from robp_interfaces.srv import Talk
import rclpy
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from typing import TypedDict
from detection_interfaces.srv import GetStuff, GetBox
from geometry_msgs.msg import Point, Pose

import tf2_geometry_msgs

from mapping_interfaces.srv import PathPlanObject, PathPlan


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
            # CheckExplorationCompletion(),
        ])


class PPPP(pt.composites.Selector):

    def __init__(self, name="PPPP"):
        super(PPPP, self).__init__(name=name, memory=True)
        self.add_children([
            # Peek(),
            pt.decorators.Inverter(name="inverter", child=Peek()),
            PPP()
        ])


class PPP(pt.composites.Sequence):
    '''
    pick, place, pop, peek, last peek to check if task done
    '''

    def __init__(self, name="PPP"):
        super(PPP, self).__init__(name=name, memory=True)
        self.add_children([
            Pick(),
            Place(),
            Pop(),
        ])


class Pop(TemplateBehaviour):
    def __init__(self, name="Pop"):
        super(Pop, self).__init__(name=name)
        self.client = self.create_client(GetStuff, '/get_stuff')

        self.register_bb('current_target_object',
                         read_access=True, write_access=False)

        self.state = pt.common.Status.RUNNING

    def initialise(self) -> None:
        super().initialise()
        self.client.wait_for_service()
        self.state = pt.common.Status.RUNNING

        request = GetStuff.Request()
        request.pop = True  # pop
        request.pop_id = self.blackboard.current_target_object['stuff_id']

        self.future = self.client.call_async(request)
        self.future.add_done_callback(self.future_callback)

    def future_callback(self, future):
        response = future.result()
        if response is None:
            self.state = pt.common.Status.FAILURE
            return

        self.state = pt.common.Status.SUCCESS
        self.future = None

        current_obj = ObjectDict()
        current_obj['stuff_id'] = response.stuff_id
        current_obj['category'] = str(response.stuff.category)
        current_obj['super_category'] = response.super_category
        current_obj['position'] = response.stuff.position.point
        self.blackboard.current_target_object = current_obj
        self.future = None
        self.state = pt.common.Status.SUCCESS

    def update(self):
        rclpy.spin_once(self, timeout_sec=0.01)
        return self.state


class Pick(pt.composites.Sequence):
    """
    for select object, go to object, fine tune object position, and pick up
    """

    def __init__(self, name="Pick"):
        super(Pick, self).__init__(name=name, memory=True)
        self.add_children([
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
            # ApproachBoxBehavior(),
            PlaceBehavior(),
        ])


class Peek(TemplateBehaviour):
    def __init__(self, name="Peek"):
        super(Peek, self).__init__(name=name)

        self.client = self.create_client(GetStuff, '/get_stuff')

        self.register_bb('current_target_object',
                         read_access=False, write_access=True)

        self.state = pt.common.Status.RUNNING

    def initialise(self) -> None:
        super().initialise()
        self.client.wait_for_service()
        self.state = pt.common.Status.RUNNING

        request = GetStuff.Request()
        request.pop = False  # peek

        self.future = self.client.call_async(request)
        self.future.add_done_callback(self.future_callback)

    def future_callback(self, future):
        response = future.result()
        if response is None:
            self.state = pt.common.Status.FAILURE
            return

        self.state = pt.common.Status.SUCCESS
        self.future = None

        # self.blackboard.current_target_object = str(response.stuff_id)
        current_obj = ObjectDict()
        current_obj['stuff_id'] = response.stuff_id
        current_obj['category'] = str(response.stuff.category)
        current_obj['super_category'] = response.super_category
        current_obj['position'] = response.stuff.position.point
        self.blackboard.current_target_object = current_obj
        self.future = None
        self.state = pt.common.Status.SUCCESS

    def update(self):
        rclpy.spin_once(self, timeout_sec=0.01)
        return self.state


class ArmToHome(TemplateBehaviour):
    def __init__(self, name="ArmToHome"):
        super().__init__(name=name)
        self.publisher_ = self.create_publisher(
            Int16MultiArray, '/multi_servo_cmd_sub', 10)
        self.subscriber_ = self.create_subscription(
            JointState, '/servo_pos_publisher', self.arm_pos_callback, 10)

        self.wild_west_talk_client = self.create_client(Talk, "talk")

        # self.register_bb('current_target_object',
        #                  read_access=True, write_access=True)

        self.current_joint_pos = [12000, 12000, 12000, 12000, 12000, 12000]
        self.position_reached = False

        self.HOME_POSITION = [12000 for i in range(12)]
        for i in range(6):
            self.HOME_POSITION[i+6] = 800
        self.HOME_POSITION[0] = 3000
        self.HOME_POSITION[1] = 12000
        self.HOME_POSITION[2] = 2000
        self.HOME_POSITION[3] = 16000
        self.HOME_POSITION[4] = 8000

    def initialise(self):
        self.publisher_.publish(Int16MultiArray(data=self.HOME_POSITION))

        talk_request = Talk.Request()
        sound_string = String()
        sound_string.data = "west"
        talk_request.sound = sound_string
        self.wild_west_talk_client.call_async(talk_request)
        # self.blackboard.set('current_target_object', '7')

    def arm_pos_callback(self, msg: JointState):
        self.current_joint_pos = msg.position
        diff = 0
        for i in range(6):
            diff += abs(self.current_joint_pos[i] - self.HOME_POSITION[i])
        self.position_reached = diff < 2000

    def update(self):

        return pt.common.Status.SUCCESS



class PlanToBoxBehavior(TemplateBehaviour):
    def __init__(self, name="PlanToBoxBehavior"):
        super(PlanToBoxBehavior, self).__init__(name=name)
        self.register_bb('current_target_box',
                         read_access=True, write_access=False)

        self.path_plan_client = self.create_client(
            PathPlan, 'path_plan')

        self.pursuit_client = ActionClient(self, Pursuit, 'pursuit')

        self.state = pt.common.Status.RUNNING

    def initialise(self) -> None:
        super().initialise()

        self.path_plan_client.wait_for_service()
        self.pursuit_client.wait_for_server()
        path_plan_request = PathPlan.Request()
        # path_plan_request.target_box_id = self.blackboard.current_target_box['box_id']
        path_plan_request.goal_pose = self.blackboard.current_target_box['pose']

        self.path_plan_future = self.path_plan_client.call_async(
            path_plan_request)
        self.path_plan_future.add_done_callback(self.path_plan_future_callback)

    def path_plan_future_callback(self, future):
        result = future.result()
        if result is None:
            self.state = pt.common.Status.FAILURE
            return

        waypoints = []
        for pose in result.path.poses:
            waypoints.insert(0, Point(
                x=pose.pose.position.x, y=pose.pose.position.y))

        goal_msg = Pursuit.Goal()
        goal_msg.waypoints = waypoints

        self.pursuit_future = self.pursuit_client.send_goal_async(goal_msg)
        self.pursuit_future.add_done_callback(self.pursuit_future_callback)

    def pursuit_future_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.state = pt.common.Status.FAILURE
            return

        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):

        result = future.result().result
        if result is None:
            self.state = pt.common.Status.FAILURE
            return

        if result.success:
            self.state = pt.common.Status.SUCCESS
        else:
            self.state = pt.common.Status.FAILURE

        self.pursuit_future = None
        self.get_result_future = None

    def update(self):
        rclpy.spin_once(self, timeout_sec=0.01)
        return self.state


class PlanToObjectBehavior(TemplateBehaviour):
    """
    plan to the object position
    """

    def __init__(self, name="PlanToObjectBehavior"):
        super().__init__(name=name)

        self.register_bb('current_target_object',
                         read_access=True, write_access=False)

        self.path_plan_client = self.create_client(
            PathPlanObject, 'path_plan_object')

        self.pursuit_client = ActionClient(self, Pursuit, 'pursuit')

        self.state = pt.common.Status.RUNNING

        # self.tf_buffer = Buffer()
        # self.tf_listener = TransformListener(
        #     self.tf_buffer, self, spin_thread=False)

    def initialise(self) -> None:
        super().initialise()

        self.path_plan_client.wait_for_service()
        self.pursuit_client.wait_for_server()
        path_plan_request = PathPlanObject.Request()
        # path_plan_request.target_object_id = int(
        #     self.blackboard.current_target_object)
        path_plan_request.target_object_id = self.blackboard.current_target_object['stuff_id']

        self.path_plan_future = self.path_plan_client.call_async(
            path_plan_request)
        self.path_plan_future.add_done_callback(self.path_plan_future_callback)

    def path_plan_future_callback(self, future):
        result = future.result()
        if result is None:
            self.state = pt.common.Status.FAILURE
            return

        waypoints = []
        # transfrom waypoints from map to odom
        # try:
        #     t = self.tf_buffer.lookup_transform(
        #         'odom', 'map', self.get_clock().now(), timeout=rclpy.time.Duration(seconds=5))
        # except Exception as e:
        #     self.get_logger().error(str(e))
        #     self.state = pt.common.Status.FAILURE
        #     self.path_plan_future = None
        #     return

        # for pose in result.path.poses:
        #     pose_odom = tf2_geometry_msgs.do_transform_pose(pose, t)
        #     waypoints.insert(0, Point(
        #         x=pose_odom.pose.position.x, y=pose_odom.pose.position.y))

        for pose in result.path.poses:
            waypoints.insert(0, Point(
                x=pose.pose.position.x, y=pose.pose.position.y))

        self.path_plan_future = None
        goal_msg = Pursuit.Goal()
        goal_msg.waypoints = waypoints

        self.pursuit_future = self.pursuit_client.send_goal_async(goal_msg)
        self.pursuit_future.add_done_callback(self.pursuit_future_callback)

    def pursuit_future_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.state = pt.common.Status.FAILURE
            return

        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result is None:
            self.state = pt.common.Status.FAILURE
            return

        if result.success:
            self.state = pt.common.Status.SUCCESS
        else:
            self.state = pt.common.Status.FAILURE

        self.pursuit_future = None
        self.get_result_future = None

    def update(self):
        # global current_object
        rclpy.spin_once(self, timeout_sec=0.01)
        return self.state


class ApproachObjectBehavior(TemplateBehaviour):
    """
    use realsense to approach the object
    """

    def __init__(self, name="ApproachObjectBehavior"):
        super().__init__(name=name)

        self.action_client = ActionClient(self, Approach, 'approach')
        # self.future = None

        # self.initialised = False

        self.change_camera_mode_pub = self.create_publisher(
            String, '/detection_ml/change_mode', 10)

        self.register_bb('current_target_object',
                         read_access=True, write_access=True)

        self.state = pt.common.Status.RUNNING

    def initialise(self) -> None:
        super().initialise()
        self.state = pt.common.Status.RUNNING

        self.change_camera_mode_pub.publish(String(data='front-camera'))

        goal_msg = Approach.Goal()
        goal_msg.target = self.blackboard.current_target_object['category']

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
            self.state = pt.common.Status.SUCCESS
            # self.state = pt.common.Status.FAILURE

        self.send_goal_future = None
        self.get_result_future = None

    def update(self):
        # global current_object

        rclpy.spin_once(self, timeout_sec=0.01)
        return self.state


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

        self.state = pt.common.Status.RUNNING
        self.camera_mode_pub.publish(String(data='arm-camera'))

        self.action_client.wait_for_server()

        goal_msg = Finetune.Goal()
        # goal_msg.object_id = self.blackboard.current_target_object['category']
        goal_msg.super_category = self.blackboard.current_target_object['super_category']
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
        return self.state

    def terminate(self, new_status: Status) -> None:
        self.camera_mode_pub.publish(String(data='front-camera'))
        return super().terminate(new_status)


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
                             -float(self.blackboard.pick_pos['y']) + 0.20]  # offset from arm_base to gripper
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


class GetBoxPositionBehavior(TemplateBehaviour):
    """
    get the box position
    """

    def __init__(self, name="GetBoxPositionBehavior"):
        super(GetBoxPositionBehavior, self).__init__(name=name)
        self.client = self.create_client(GetBox, '/get_box')

        self.register_bb('current_target_box',
                         read_access=False, write_access=True)
        self.register_bb('current_target_object',
                         read_access=True, write_access=False)

        self.state = pt.common.Status.RUNNING

    def initialise(self) -> None:
        super().initialise()
        self.client.wait_for_service()
        self.state = pt.common.Status.RUNNING

        request = GetBox.Request()
        request.box_id = super_category_to_box_id[self.blackboard.current_target_object['super_category']]

        self.future = self.client.call_async(request)
        self.future.add_done_callback(self.future_callback)

    def future_callback(self, future):
        response = future.result()
        if response is None or response.success is False:
            self.state = pt.common.Status.FAILURE
            return

        self.state = pt.common.Status.SUCCESS
        self.future = None

        current_box = BoxDict()
        current_box['box_id'] = super_category_to_box_id[self.blackboard.current_target_object['super_category']]
        current_box['pose'] = response.box_pose.pose

        self.future = None
        self.state = pt.common.Status.SUCCESS

    def update(self):
        rclpy.spin_once(self, timeout_sec=0.01)
        return self.state


class PlaceBehavior(TemplateBehaviour):
    def __init__(self, name="PlaceBehavior"):
        super(PlaceBehavior, self).__init__(name=name)

        self.action_client = ActionClient(self, Arm, 'arm')

        self.state = pt.common.Status.RUNNING

    def initialise(self) -> None:
        super().initialise()
        goal_msg = Arm.Goal()
        goal_msg.command = "place"
        goal_msg.position = [0.0, 0.0]
        goal_msg.angle = 0.0

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


class CheckTaskCompletion(pt.behaviour.Behaviour):
    def __init__(self, name="CheckTaskCompletion"):
        super(CheckTaskCompletion, self).__init__(name=name)

    def update(self):
        # place holder for the check task completion behavior
        return pt.common.Status.RUNNING


class ExplorePointBehavior(TemplateBehaviour):
    def __init__(self, name="ExplorePointBehavior"):
        super(ExplorePointBehavior, self).__init__(name=name)

        self.action_client = ActionClient(self, Explore, 'explore')

        self.state = pt.common.Status.RUNNING

    def initialise(self) -> None:
        super().initialise()

        goal_msg = Explore.Goal()

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


class CheckExplorationCompletion(pt.behaviour.Behaviour):
    def __init__(self, name="CheckExplorationCompletion"):
        super(CheckExplorationCompletion, self).__init__(name=name)

    def update(self):
        # place holder for the check exploration completion behavior
        return pt.common.Status.SUCCESS


class Done(pt.behaviour.Behaviour):
    '''
    always return running
    '''

    def __init__(self, name="Done"):
        super(Done, self).__init__(name=name)

    def update(self):
        return pt.common.Status.RUNNING


class PickPosDict(TypedDict):
    # x and y in arm camera frame
    x: float
    y: float
    # angle measured clockwise from camera vertical axis
    angle: float


class ObjectDict(TypedDict):
    stuff_id: int
    category: str
    super_category: str
    position: Point


class BoxDict(TypedDict):
    box_id: int
    pose: Pose


# TODO: not sure if this is correct
super_category_to_box_id = {
    "cube": 1,
    "sphere": 2,
    "animal": 3,
}
