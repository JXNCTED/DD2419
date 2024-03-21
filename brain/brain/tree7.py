import sys

import rclpy
from std_msgs.msg import Bool

import py_trees as pt
import py_trees_ros as ptr
import py_trees.console as console


def tutorial_create_root() -> pt.behaviour.Behaviour:
    """
    Insert a task between battery emergency and idle behaviours that
    controls a rotation action controller and notifications simultaenously
    to scan a room.

    Returns:
        the root of the tree
    """
    root = pt.composites.Parallel(
        name="Tutorial Eight",
        policy=pt.common.ParallelPolicy.SuccessOnAll(
            synchronise=False
        )
    )

    topics2bb = pt.composites.Sequence("Topics2BB")
    scan2bb = ptr.subscribers.EventToBlackboard(
        name="Scan2BB",
        topic_name="/dashboard/scan",
        qos_profile=ptr.utilities.qos_profile_unlatched(),
        variable_name="event_scan_button"
    )
    cancel2bb = ptr.subscribers.EventToBlackboard(
        name="Cancel2BB",
        topic_name="/dashboard/cancel",
        qos_profile=ptr.utilities.qos_profile_unlatched(),
        variable_name="event_cancel_button"
    )
    battery2bb = ptr.battery.ToBlackboard(
        name="Battery2BB",
        topic_name="/battery/state",
        qos_profile=ptr.utilities.qos_profile_unlatched(),
        threshold=30.0
    )
    tasks = pt.composites.Selector("Tasks")
    flash_red = behaviours.FlashLedStrip(
        name="Flash Red",
        colour="red"
    )

    # Emergency Tasks
    def check_battery_low_on_blackboard(blackboard: pt.blackboard.Blackboard) -> bool:
        return blackboard.battery_low_warning

    battery_emergency = pt.decorators.EternalGuard(
        name="Battery Low?",
        condition=check_battery_low_on_blackboard,
        blackboard_keys={"battery_low_warning"},
        child=flash_red
    )

    # Fallback task
    idle = pt.behaviours.Running(name="Idle")

    root.add_child(topics2bb)
    topics2bb.add_children([scan2bb, cancel2bb, battery2bb])
    root.add_child(tasks)
    tasks.add_children([battery_emergency, idle])
    return root


class CheckForObjects(ptr.behaviour.Behaviour):
    def __init__(self, name: str):
        """
        Create the core tree
        """
        super(CheckForObjects, self).__init__(
            name=name
        )
        self.is_object_centered = False
        self.latest_message = None

    def setup(self, **kwargs):
        self.node = kwargs['node']
        # Have random topic here. THis topic will have true or false values for if the object is in the middle.
        self.is_object_centered_sub = self.node.create_subscription(
            Bool, "/is_object_centered", self.is_object_centered_callback, 10)

    def update(self):
        # Manually read from the topic
        # Non-blocking read. Doesn't work...

        if self.latest_message:
            self.is_object_centered_callback(self.latest_message)

        print(self.is_object_centered)

        if self.is_object_centered:
            print("Object is centered")
            return pt.common.Status.SUCCESS
        else:
            print("Object is not centered")
            return pt.common.Status.RUNNING

    def is_object_centered_callback(self, msg):
        try:
            print(msg.data)
            self.is_object_centered = msg.data
            self.latest_message = msg
            print("Received message:", msg.data)
        except Exception as e:
            print("Error in callback:", e)


def main():
    """
    Entry point for the demo script.
    """
    rclpy.init(args=None)
    tree = DynamicApplicationTree()
    try:
        tree.setup(timeout=15)
    except ptr.exceptions.TimedOutError as e:
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
