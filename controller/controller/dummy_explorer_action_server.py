#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node

from rclpy.action import ActionServer, GoalResponse, CancelResponse
from robp_interfaces.action import Explore
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Twist
from detection_interfaces.msg import BoxList, StuffList, Stuff, Box

from sensor_msgs.msg import Joy


super_category_to_box_id = {
    "none": 0,
    "cube": 1,
    "sphere": 2,
    "animal": 3,
}


class DummyExplorer(Node):
    def __init__(self):
        super().__init__('dummy_explorer')

        self.done = False
        self.v = 0.0
        self.w = 0.0

        self.stuff_set = set()
        self.box_set = set()

        self.cb_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self,
            Explore,
            'explore',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,

        )

        self.rate = self.create_rate(100)

        self._joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10,
            callback_group=self.cb_group

        )

        self._stuff_sub = self.create_subscription(
            StuffList,
            '/category_eval/stuff_list',
            self.stuff_callback,
            10,
            callback_group=self.cb_group
        )

        self._box_sub = self.create_subscription(
            BoxList,
            '/box_list',
            self.box_callback,
            10,
            callback_group=self.cb_group
        )


        self.twist_pub_ = self.create_publisher(
            Twist,
            '/motor_controller/twist',
            10,
            callback_group=self.cb_group
        )

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        self.done = False
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.done = True
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        result = Explore.Result()
        result.success = False

        while rclpy.ok() and not self.done:
            twist_pub_msg = Twist()
            twist_pub_msg.angular.z = self.w
            twist_pub_msg.linear.x = self.v

            self.twist_pub_.publish(twist_pub_msg)

            self.rate.sleep()

        for _ in range(10):
            twist_pub_msg = Twist()
            twist_pub_msg.angular.z = 0.0
            twist_pub_msg.linear.x = 0.0
            self.twist_pub_.publish(twist_pub_msg)
            self.rate.sleep()

        result.success = True
        goal_handle.succeed()

        self.get_logger().info('\033[92mGoal succeeded\033[0m')

        return result

    def joy_callback(self, msg: Joy):
        self.v = 0.5 * msg.axes[1]
        self.w = 1.0 * msg.axes[3]

    def stuff_callback(self, msg: StuffList):
        stuff: Stuff
        if len(msg.data) == 0:
            return
        for stuff in msg.data:
            if stuff is not None:
                self.stuff_set.add(super_category_to_box_id[stuff.super_category])
        
        # if the intersection is not empty, then we are done with the exploration
        self.done = len(self.stuff_set.intersection(self.box_set)) > 0
    
    def box_callback(self, msg: BoxList):
        if len(msg.boxes) == 0:
            return
        box: Box
        for box in msg.boxes:
            if box is not None:
                self.box_set.add(box.aruco_id)



def main():
    rclpy.init()
    node = DummyExplorer()
    executor = rclpy.executors.MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor)
    except KeyboardInterrupt:
        pass
    node.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


