import rclpy
from rclpy.node import Node
import py_trees
import py_trees_ros
import py_trees_ros_interfaces.msg as py_trees_msgs
from std_msgs.msg import Bool


# class BehaviorTreeController(Node):
#     def __init__(self):
#         super().__init__('behavior_tree_controller')
#         self.create_subscription(
#             Bool,
#             '/test_bool_sub',
#             self.status_callback,
#             10 getTopicInfo = py_trees_ros.subscribers.EventToBlackboard(


#         )
#         self.publisher_ = self.create_publisher(
#             Bool,
#             '/test_bool_pub',
#             10
#         )

#         # Initialize the blackboard
#         self.blackboard = py_trees.blackboard.Blackboard()
#         # Set initial data on the blackboard
#         self.blackboard.set('test_value', 42)
#         self.tree = self.create_behavior_tree()
#         # Set the blackboard for the behavior tree
#         self.tree.blackboard = self.blackboard

#     def create_behavior_tree(self):
#         # Define behaviors
#         class SetBlackboardValue(py_trees.behaviour.Behaviour):
#             def __init__(self, name):
#                 super().__init__(name=name)

#             def update(self):
#                 self.blackboard.set('tree_value', 99)
#                 return py_trees.common.Status.SUCCESS

#         class GetBlackboardValue(py_trees.behaviour.Behaviour):
#             def __init__(self, name):
#                 super().__init__(name=name)

#             def update(self):
#                 value = self.blackboard.get('test_value')
#                 print("Test Value:", value)
#                 tree_value = self.blackboard.get('tree_value')
#                 print("Tree Value:", tree_value)
#                 return py_trees.common.Status.SUCCESS

#         # Create the tree
#         root = py_trees.composites.Sequence(name="Root", memory=False)
#         set_value = SetBlackboardValue(name="Set Blackboard Value")
#         get_value = GetBlackboardValue(name="Get Blackboard Value")
#         root.add_children([set_value, get_value])

#         tree = py_trees_ros.trees.BehaviourTree(root)
#         return tree

#     def status_callback(self, msg):
#         # Logic to handle received status messages if needed
#         pass

#     def publish_control_message(self, command):
#         msg = py_trees_msgs.BehaviorTreeStatus()
#         # Populate message with appropriate data based on command
#         self.publisher_.publish(msg)


# def main(args=None):
#     rclpy.init(args=args)
#     controller = BehaviorTreeController()
#     try:
#         rclpy.spin(controller)
#     finally:
#         controller.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()


""" Hi Sean! 
I have been doing some work on the tree, scaling down the tutorial code so we can insert our code and be off to the races. 


From what I understand of this, is that the tree right now is relient on listening to topics in order to progress through the tree.
    However, it seems that the tree is already set to all true which means it finishes as soon as the tree starts.Cunt ar gcúl.. Hmmm.
    I've been really trying to create some kind of node that fetches data from a topic (/dist_bool for instance) and then returning success
    if the data is true. But I seem to not get it to work every time.

    That is the biggest challenge right now, to incorporate a node (see CheckDistBool below) into the tree that fetches data through a topic and then returns success if the data is true.
    If we have that we can probably construct the entire tree.

    Deireadh seachtaine maith agat!
"""
#!/usr/bin/env python3

import sys
import operator
import py_trees
import py_trees_ros.trees
import py_trees.console as console
import py_trees_ros_interfaces.action as py_trees_actions  # noqa
import py_trees_ros_interfaces.srv as py_trees_srvs  # noqa
import rclpy
from rclpy.node import Node
import std_msgs.msg as std_msgs
import rcl_interfaces.msg as rcl_msgs
import rcl_interfaces.srv as rcl_srvs
from std_msgs.msg import Bool


def tutorial_create_root() -> py_trees.behaviour.Behaviour:

    root = py_trees.composites.Parallel(
        name="Tutorial Eight",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(
            # synchronise=False
        )
    )

    topics2bb = py_trees.composites.Sequence("Topics2BB", False)

    # Should be able to get messages from topic and show them in the blackboard. However this is always set to true???
    wait4detect = py_trees_ros.subscribers.EventToBlackboard(
        name="Wait for detection?",
        topic_name="/shit_topic",
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        variable_name="WaitForDetection"
    )
    # Display on blackboard. Only returns true?
    cancel2bb = py_trees_ros.subscribers.EventToBlackboard(
        name="Cancel2BB",
        topic_name="/dashboard/cancel",
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        variable_name="event_cancel_button"
    )

    # Display on blackboard
    getDist = py_trees_ros.subscribers.EventToBlackboard(
        name="getDist?",
        topic_name="/dist_bool",
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        variable_name="getDist"
    )

    flash_red = py_trees_ros.subscribers.EventToBlackboard(
        name="Flash ?",
        topic_name="/dist_bool",
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        variable_name="Flash"
    )

    def check_battery_low_on_blackboard(blackboard: py_trees.blackboard.Blackboard) -> bool:
        # Returning true to continue to the flash_red.
        return True

    # Eternalguard Continuously guard (with a condition) the execution of a child/subtree.The ernal guard checks a condition prior to *every* tick of the child/subtree. If at any time the condition fails, the child/subtree is invalidated
    battery_emergency = py_trees.decorators.EternalGuard(
        name="Battery Emergency",
        condition=check_battery_low_on_blackboard,
        blackboard_keys={"battery_low_warning"},
        child=flash_red
    )

    # Fallback task
    idle = py_trees.behaviours.Running(name="Idle")

    root.add_child(topics2bb)
    topics2bb.add_children(
        [wait4detect, cancel2bb, getDist, battery_emergency, idle])

    return root


class CheckDistBool(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, node, topic_name: str = "/dist_bool"):
        super(CheckDistBool, self).__init__(name=name)
        self.topic_name = "/dist_bool"
        self.node = node
        self.subscriber = None
        self.received_value = None
        self.qos_profile = rclpy.qos.qos_profile_sensor_data

    def setup(self, **kwargs):
        self.subscriber = self.node.create_subscription(self,
                                                        msg_type=Bool,
                                                        topic="/dist_bool",
                                                        callback=self.dist_bool_callback,  # Provide your callback function here
                                                        qos_profile=self.qos_profile  # Adjust the QoS profile as per your requirements
                                                        )

    def dist_bool_callback(self, msg):
        self.received_value = msg.data

    def update(self):
        if self.received_value is None:
            return py_trees.common.Status.RUNNING
        elif self.received_value:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE


class DynamicApplicationTree(py_trees_ros.trees.BehaviourTree):

    def __init__(self):

        super().__init__(
            root=tutorial_create_root(),
            unicode_tree_debug=True,
        )
        self.add_post_tick_handler(
            self.prune_application_subtree_if_done
        )

    def setup(self, timeout: float):
        super().setup(timeout=timeout)
        self._report_service = self.node.create_service(
            srv_type=py_trees_srvs.StatusReport,
            srv_name="~/report",
            callback=None,
            qos_profile=rclpy.qos.qos_profile_services_default
        )
        self._job_subscriber = self.node.create_subscription(
            msg_type=std_msgs.Empty,
            topic="/dashboard/scan",
            callback=None,
            qos_profile=py_trees_ros.utilities.qos_profile_unlatched()
        )

    def prune_application_subtree_if_done(self, tree):

        # executing
        if self.busy():
            job = self.priorities.children[-2]
            # finished
            if job.status == py_trees.common.Status.SUCCESS or job.status == py_trees.common.Status.FAILURE:
                self.node.get_logger().info(
                    "{0}: finished [{1}]".format(job.name, job.status))
                for node in job.iterate():
                    node.shutdown()
                tree.prune_subtree(job.id)

    def busy(self):

        return len(self.priorities.children) == 3

    @property
    def priorities(self) -> py_trees.composites.Selector:
        return self.root.children[-1]


def main():

    rclpy.init(args=None)
    tree = DynamicApplicationTree()
    try:
        tree.setup(timeout=15)
    except py_trees_ros.exceptions.TimedOutError as e:
        console.logerror(
            console.red + "failed to setup the tree, aborting [{}]".format(str(e)) + console.reset)
        tree.shutdown()
        rclpy.shutdown()
        sys.exit(1)
    except KeyboardInterrupt:
        # not a warning, nor error, usually a user-initiated shutdown
        console.logerror("tree setup interrupted")
        tree.shutdown()
        rclpy.shutdown()
        sys.exit(1)

    tree.tick_tock(period_ms=1000.0)

    try:
        rclpy.spin(tree.node)
    except KeyboardInterrupt:
        pass

    tree.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
