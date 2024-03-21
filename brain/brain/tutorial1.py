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


class FlashLedStrip(py_trees.behaviour.Behaviour):
    """
    This behaviour simply shoots a command off to the LEDStrip to flash
    a certain colour and returns :attr:`~py_trees.common.Status.RUNNING`.
    Note that this behaviour will never return with
    :attr:`~py_trees.common.Status.SUCCESS` but will send a clearing
    command to the LEDStrip if it is cancelled or interrupted by a higher
    priority behaviour.

    Publishers:
        * **/led_strip/command** (:class:`std_msgs.msg.String`)

          * colourised string command for the led strip ['red', 'green', 'blue']

    Args:
        name: name of the behaviour
        topic_name : name of the battery state topic
        colour: colour to flash ['red', 'green', blue']
    """

    def __init__(
            self,
            name: str,
            topic_name: str = "/led_strip/command",
            colour: str = "red"
    ):
        super(FlashLedStrip, self).__init__(name=name)
        self.topic_name = topic_name
        self.colour = colour

    def setup(self, **kwargs):
        """
        Setup the publisher which will stream commands to the mock robot.

        Args:
            **kwargs (:obj:`dict`): look for the 'node' object being passed down from the tree

        Raises:
            :class:`KeyError`: if a ros2 node isn't passed under the key 'node' in kwargs
        """
        self.logger.debug("{}.setup()".format(self.qualified_name))
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
                self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability

        self.publisher = self.node.create_publisher(
            msg_type=std_msgs.String,
            topic=self.topic_name,
            qos_profile=py_trees_ros.utilities.qos_profile_latched()
        )
        self.feedback_message = "publisher created"

    def update(self) -> py_trees.common.Status:
        """
        Annoy the led strip to keep firing every time it ticks over (the led strip will clear itself
        if no command is forthcoming within a certain period of time).
        This behaviour will only finish if it is terminated or priority interrupted from above.

        Returns:
            Always returns :attr:`~py_trees.common.Status.RUNNING`
        """
        self.logger.debug("%s.update()" % self.__class__.__name__)
        self.publisher.publish(std_msgs.String(data=self.colour))
        self.feedback_message = "flashing {0}".format(self.colour)
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status):
        """
        Shoot off a clearing command to the led strip.

        Args:
            new_status: the behaviour is transitioning to this new status
        """
        self.logger.debug(
            "{}.terminate({})".format(
                self.qualified_name,
                "{}->{}".format(self.status,
                                new_status) if self.status != new_status else "{}".format(new_status)
            )
        )
        self.publisher.publish(std_msgs.String(data=""))
        self.feedback_message = "cleared"


class ScanContext(py_trees.behaviour.Behaviour):
    """
    Alludes to switching the context of the runtime system for a scanning
    action. Technically, it reaches out to the mock robots safety sensor
    dynamic parameter, switches it off in :meth:`initialise()` and maintains
    that for the the duration of the context before returning it to
    it's original value in :meth:`terminate()`.

    Args:
        name (:obj:`str`): name of the behaviour
    """

    def __init__(self, name):
        super().__init__(name=name)

        self.cached_context = None

    def setup(self, **kwargs):
        """
        Setup the ros2 communications infrastructure.

        Args:
            **kwargs (:obj:`dict`): look for the 'node' object being passed down from the tree

        Raises:
            :class:`KeyError`: if a ros2 node isn't passed under the key 'node' in kwargs
        """
        self.logger.debug("%s.setup()" % self.__class__.__name__)

        # ros2 node
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
                self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability

        # parameter service clients
        self.parameter_clients = {
            'get_safety_sensors': self.node.create_client(
                rcl_srvs.GetParameters,
                '/safety_sensors/get_parameters'
            ),
            'set_safety_sensors': self.node.create_client(
                rcl_srvs.SetParameters,
                '/safety_sensors/set_parameters'
            )
        }
        for name, client in self.parameter_clients.items():
            if not client.wait_for_service(timeout_sec=3.0):
                raise RuntimeError(
                    "client timed out waiting for server [{}]".format(name))

    def initialise(self):
        """
        Reset the cached context and trigger the chain of get/set parameter
        calls involved in changing the context.

        .. note::

           Completing the chain of service calls here
           (with `rclpy.spin_until_future_complete(node, future)`)
           is not possible if this behaviour is encapsulated inside, e.g.
           a tree tick activated by a ros2 timer callback, since it is
           already part of a scheduled job in a spinning node. It will
           just deadlock.

           Prefer instead to chain a sequence of events that will be
           completed over a span of ticks instead of at best, blocking
           here and at worst, falling into deadlock.

        """
        self.logger.debug("%s.initialise()" % self.__class__.__name__)
        self.cached_context = None
        # kickstart get/set parameter chain
        self._send_get_parameter_request()

    def update(self) -> py_trees.common.Status:
        """
        Complete the chain of calls begun in :meth:`initialise()` and then
        maintain the context (i.e. :class:`py_trees.behaviour.Behaviour` and
        return :data:`~py_trees.common.Status.RUNNING`).
        """
        self.logger.debug("%s.update()" % self.__class__.__name__)
        all_done = False

        # wait for get_parameter to return
        if self.cached_context is None:
            if self._process_get_parameter_response():
                self._send_set_parameter_request(value=True)
            return py_trees.common.Status.RUNNING

        # wait for set parameter to return
        if not all_done:
            if self._process_set_parameter_response():
                all_done = True
            return py_trees.common.Status.RUNNING

        # just spin around, wait for an interrupt to trigger terminate
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status):
        """
        Reset the parameters back to their original (cached) values.

        Args:
            new_status: the behaviour is transitioning to this new status
        """
        self.logger.debug("%s.terminate(%s)" % (self.__class__.__name__, "%s->%s" % (
            self.status, new_status) if self.status != new_status else "%s" % new_status))
        if (
            new_status == py_trees.common.Status.INVALID and
            self.cached_context is not None
        ):
            self._send_set_parameter_request(value=self.cached_context)
            # don't worry about the response, no chance to catch it anyway

    def _send_get_parameter_request(self):
        request = rcl_srvs.GetParameters.Request()  # noqa
        request.names.append("enabled")
        self.get_parameter_future = self.parameter_clients['get_safety_sensors'].call_async(
            request)

    def _process_get_parameter_response(self) -> bool:
        if not self.get_parameter_future.done():
            return False
        if self.get_parameter_future.result() is None:
            self.feedback_message = "failed to retrieve the safety sensors context"
            self.node.get_logger().error(self.feedback_message)
            # self.node.get_logger().info('Service call failed %r' % (future.exception(),))
            raise RuntimeError(self.feedback_message)
        if len(self.get_parameter_future.result().values) > 1:
            self.feedback_message = "expected one parameter value, got multiple [{}]".format(
                "/safety_sensors/enabled")
            raise RuntimeError(self.feedback_message)
        value = self.get_parameter_future.result().values[0]
        if value.type != rcl_msgs.ParameterType.PARAMETER_BOOL:  # noqa
            self.feedback_message = "expected parameter type bool, got [{}]{}]".format(
                value.type, "/safety_sensors/enabled")
            self.node.get_logger().error(self.feedback_message)
            raise RuntimeError(self.feedback_message)
        self.cached_context = value.bool_value
        return True

    def _send_set_parameter_request(self, value: bool):
        request = rcl_srvs.SetParameters.Request()  # noqa
        parameter = rcl_msgs.Parameter()
        parameter.name = "enabled"
        parameter.value.type = rcl_msgs.ParameterType.PARAMETER_BOOL  # noqa
        parameter.value.bool_value = value
        request.parameters.append(parameter)
        self.set_parameter_future = self.parameter_clients['set_safety_sensors'].call_async(
            request)

    def _process_set_parameter_response(self) -> bool:
        if not self.get_parameter_future.done():
            return False
        if self.set_parameter_future.result() is not None:
            self.feedback_message = "reconfigured the safety sensors context"
        else:
            self.feedback_message = "failed to reconfigure the safety sensors context"
            self.node.get_logger().error(self.feedback_message)
            # self.node.get_logger().info('service call failed %r' % (future.exception(),))
        return True


def tutorial_create_scan_subtree() -> py_trees.behaviour.Behaviour:
    """
    Create the job subtree based on the incoming goal specification.
    Args:
        goal (:class:`~std_msgs.msg.Empty`): incoming goal specification
    Returns:
       :class:`~py_trees.behaviour.Behaviour`: subtree root
    """
    # behaviours
    scan = py_trees.composites.Sequence(name="Scan")
    scan_or_die = py_trees.composites.Selector(name="Scan or Die")
    die = py_trees.composites.Sequence(name="Die")
    failed_notification = py_trees.composites.Parallel(
        name="Notification",
        policy=py_trees.common.ParallelPolicy.SuccessOnOne()
    )
    failed_flash_green = FlashLedStrip(
        name="Flash Red", colour="red")
    failed_pause = py_trees.timers.Timer("Pause", duration=3.0)
    result_failed_to_bb = py_trees.behaviours.SetBlackboardVariable(
        name="Result2BB\n'failed'",
        variable_name='scan_result',
        variable_value='failed'
    )
    ere_we_go = py_trees.composites.Sequence(name="Ere we Go")
    undock = py_trees_ros.actions.ActionClient(
        name="UnDock",
        action_type=py_trees_actions.Dock,
        action_name="dock",
        action_goal=py_trees_actions.Dock.Goal(dock=False),  # noqa
        generate_feedback_message=lambda msg: "undocking"
    )
    scan_or_be_cancelled = py_trees.composites.Selector("Scan or Be Cancelled")
    cancelling = py_trees.composites.Sequence("Cancelling?")
    is_cancel_requested = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Cancel?",
        check=py_trees.common.ComparisonExpression(
            variable="event_cancel_button",
            value=True,
            operator=operator.eq
        )
    )
    move_home_after_cancel = py_trees_ros.actions.ActionClient(
        name="Move Home",
        action_type=py_trees_actions.MoveBase,
        action_name="move_base",
        action_goal=py_trees_actions.MoveBase.Goal(),  # noqa
        generate_feedback_message=lambda msg: "moving home"
    )
    result_cancelled_to_bb = py_trees.behaviours.SetBlackboardVariable(
        name="Result2BB\n'cancelled'",
        variable_name='scan_result',
        variable_value='cancelled'
    )
    move_out_and_scan = py_trees.composites.Sequence("Move Out and Scan")
    move_base = py_trees_ros.actions.ActionClient(
        name="Move Out",
        action_type=py_trees_actions.MoveBase,
        action_name="move_base",
        action_goal=py_trees_actions.MoveBase.Goal(),  # noqa
        generate_feedback_message=lambda msg: "moving out"
    )
    scanning = py_trees.composites.Parallel(
        name="Scanning",
        policy=py_trees.common.ParallelPolicy.SuccessOnOne()
    )
    scan_context_switch = ScanContext("Context Switch")
    scan_rotate = py_trees_ros.actions.ActionClient(
        name="Rotate",
        action_type=py_trees_actions.Rotate,
        action_name="rotate",
        action_goal=py_trees_actions.Rotate.Goal(),  # noqa
        generate_feedback_message=lambda msg: "{:.2f}%%".format(
            msg.feedback.percentage_completed)
    )
    scan_flash_blue = FlashLedStrip(
        name="Flash Blue", colour="blue")
    move_home_after_scan = py_trees_ros.actions.ActionClient(
        name="Move Home",
        action_type=py_trees_actions.MoveBase,
        action_name="move_base",
        action_goal=py_trees_actions.MoveBase.Goal(),  # noqa
        generate_feedback_message=lambda msg: "moving home"
    )
    result_succeeded_to_bb = py_trees.behaviours.SetBlackboardVariable(
        name="Result2BB\n'succeeded'",
        variable_name='scan_result',
        variable_value='succeeded'
    )
    celebrate = py_trees.composites.Parallel(
        name="Celebrate",
        policy=py_trees.common.ParallelPolicy.SuccessOnOne()
    )
    celebrate_flash_green = FlashLedStrip(
        name="Flash Green", colour="green")
    celebrate_pause = py_trees.timers.Timer("Pause", duration=3.0)
    dock = py_trees_ros.actions.ActionClient(
        name="Dock",
        action_type=py_trees_actions.Dock,
        action_name="dock",
        action_goal=py_trees_actions.Dock.Goal(dock=True),  # noqa
        generate_feedback_message=lambda msg: "docking"
    )

    class SendResult(py_trees.behaviour.Behaviour):

        def __init__(self, name: str):
            super().__init__(name="Send Result")
            self.blackboard = self.attach_blackboard_client(name=self.name)
            self.blackboard.register_key(
                key="scan_result",
                access=py_trees.common.Access.READ
            )

        def update(self):
            print(console.green +
                  "********** Result: {} **********".format(self.blackboard.scan_result) +
                  console.reset
                  )
            return py_trees.common.Status.SUCCESS

    send_result = SendResult(name="Send Result")

    scan.add_children([scan_or_die, send_result])
    scan_or_die.add_children([ere_we_go, die])
    die.add_children([failed_notification, result_failed_to_bb])
    failed_notification.add_children([failed_flash_green, failed_pause])
    ere_we_go.add_children([undock, scan_or_be_cancelled, dock, celebrate])
    scan_or_be_cancelled.add_children([cancelling, move_out_and_scan])
    cancelling.add_children(
        [is_cancel_requested, move_home_after_cancel, result_cancelled_to_bb])
    move_out_and_scan.add_children(
        [move_base, scanning, move_home_after_scan, result_succeeded_to_bb])
    scanning.add_children([scan_context_switch, scan_rotate, scan_flash_blue])
    celebrate.add_children([celebrate_flash_green, celebrate_pause])
    return scan


def tutorial_create_root() -> py_trees.behaviour.Behaviour:
    """
    Insert a task between battery emergency and idle behaviours that
    controls a rotation action controller and notifications simultaenously
    to scan a room.

    Returns:
        the root of the tree
    """
    root = py_trees.composites.Parallel(
        name="Tutorial Eight",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(
            synchronise=False
        )
    )

    topics2bb = py_trees.composites.Sequence("Topics2BB", False)
    scan2bb = py_trees_ros.subscribers.EventToBlackboard(
        name="Scan2BB",
        topic_name="/dashboard/scan",
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        variable_name="event_scan_button"
    )
    cancel2bb = py_trees_ros.subscribers.EventToBlackboard(
        name="Cancel2BB",
        topic_name="/dashboard/cancel",
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        variable_name="event_cancel_button"
    )
    battery2bb = py_trees_ros.battery.ToBlackboard(
        name="Battery2BB",
        topic_name="/battery/state",
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        threshold=30.0
    )
    tasks = py_trees.composites.Selector("Tasks", False)
    flash_red = FlashLedStrip(
        name="Flash Red",
        colour="red"
    )

    # Emergency Tasks
    def check_battery_low_on_blackboard(blackboard: py_trees.blackboard.Blackboard) -> bool:
        return blackboard.battery_low_warning

    battery_emergency = py_trees.decorators.EternalGuard(
        name="Battery Low?",
        condition=check_battery_low_on_blackboard,
        blackboard_keys={"battery_low_warning"},
        child=flash_red
    )

    # Fallback task
    idle = py_trees.behaviours.Running(name="Idle")

    root.add_child(topics2bb)
    topics2bb.add_children([scan2bb, cancel2bb, battery2bb])
    root.add_child(tasks)
    tasks.add_children([battery_emergency, idle])
    return root


class DynamicApplicationTree(py_trees_ros.trees.BehaviourTree):
    """
    Wraps the ROS behaviour tree manager in a class that manages loading
    and unloading of jobs.
    """

    def __init__(self):
        """
        Create the core tree and add post tick handlers for post-execution
        management of the tree.
        """
        super().__init__(
            root=tutorial_create_root(),
            unicode_tree_debug=True
        )
        self.add_post_tick_handler(
            self.prune_application_subtree_if_done
        )

    def setup(self, timeout: float):
        """
        Setup the tree and connect additional application management / status
        report subscribers and services.

        Args:
            timeout: time (s) to wait (use common.Duration.INFINITE to block indefinitely)
        """
        super().setup(timeout=timeout)
        self._report_service = self.node.create_service(
            srv_type=py_trees_srvs.StatusReport,
            srv_name="~/report",
            callback=self.deliver_status_report,
            qos_profile=rclpy.qos.qos_profile_services_default
        )
        self._job_subscriber = self.node.create_subscription(
            msg_type=std_msgs.Empty,
            topic="/dashboard/scan",
            callback=self.receive_incoming_job,
            qos_profile=py_trees_ros.utilities.qos_profile_unlatched()
        )

    def receive_incoming_job(self, msg: std_msgs.Empty):
        """
        Incoming job callback.

        Args:
            msg: incoming goal message

        Raises:
            Exception: be ready to catch if any of the behaviours raise an exception
        """

        if self.busy():
            self.node.get_logger().warning("rejecting new job, last job is still active")
        else:
            scan_subtree = tutorial_create_scan_subtree()
            try:
                py_trees.trees.setup(
                    root=scan_subtree,
                    node=self.node
                )
            except Exception as e:
                console.logerror(
                    console.red + "failed to setup the scan subtree, aborting [{}]".format(str(e)) + console.reset)
                sys.exit(1)
            self.insert_subtree(scan_subtree, self.priorities.id, 1)
            self.node.get_logger().info("inserted job subtree")

    def deliver_status_report(
            self,
            unused_request: py_trees_srvs.StatusReport.Request,  # noqa
            response: py_trees_srvs.StatusReport.Response  # noqa
         ):
        """
        Prepare a status report for an external service client.

        Args:
            unused_request: empty request message
        """
        # last result value or none
        last_result = self.blackboard_exchange.blackboard.get(
            name="scan_result")
        if self.busy():
            response.report = "executing"
        elif self.root.tip().has_parent_with_name("Battery Emergency"):
            response.report = "battery [last result: {}]".format(last_result)
        else:
            response.report = "idle [last result: {}]".format(last_result)
        return response

    def prune_application_subtree_if_done(self, tree):
        """
        Check if a job is running and if it has finished. If so, prune the job subtree from the tree.
        Additionally, make a status report upon introspection of the tree.
        Args:
            tree (:class:`~py_trees.trees.BehaviourTree`): tree to investigate/manipulate.
        """
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
        """
        Check if a job subtree exists and is running. Only one job is permitted at
        a time, so it is sufficient to just check that the priority task selector
        is of length three (note: there is always emergency and idle tasks
        alongside the active job). When the job is not active, it is
        pruned from the tree, leaving just two prioritised tasks (emergency and idle).

        Returns:
            :obj:`bool`: whether it is busy with a job subtree or not
        """
        return len(self.priorities.children) == 3

    @property
    def priorities(self) -> py_trees.composites.Selector:
        """
        Returns the composite (:class:`~py_trees.composites.Selector`) that is
        home to the prioritised list of tasks.
        """
        return self.root.children[-1]
    

class SimpleNode(Node):
    def __init__(self):
        super().__init__('simple_node')
        self.get_logger().info('Simple Node Initialized')

def main():
    """
    Entry point for the demo script.
    """
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
