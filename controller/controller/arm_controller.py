#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import JointState
import queue


class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')

        self.action_queue = queue.Queue()
        self.current_joint_pos = [12000, 12000, 12000, 12000, 12000, 12000]
        self.target_joint_pos = [12000, 12000, 12000, 12000, 12000, 12000]

        self.position_reached = True

        self.timeoutCnt = 0

        self.arm_pub_ = self.create_publisher(
            Int16MultiArray, '/multi_servo_cmd_sub', 10)
        self.arm_pos_sub_ = self.create_subscription(
            JointState, '/servo_pos_publisher', self.arm_pos_callback, 10)
        self.arm_joint_space_target_sub = self.create_subscription(
            JointState, '/servo_joint_space_target', self.joint_space_target_callback, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def arm_pos_callback(self, msg: JointState):
        self.current_joint_pos = msg.position
        diff = 0
        for i in range(6):
            diff += abs(self.current_joint_pos[i] - self.target_joint_pos[i])
        self.position_reached = diff < 2000
        print(f"diff:{diff} {self.position_reached}")

    def joint_space_target_callback(self, msg: JointState):
        QUEUE_SIZE = 30
        if self.action_queue.qsize() <= QUEUE_SIZE:
            self.action_queue.put(msg)

    def timer_callback(self):
        if (self.position_reached and not self.action_queue.empty()):
            next_target = self.action_queue.get()
            self.target_joint_pos = next_target.position

            arm_pub_msg = Int16MultiArray()
            arm_pub_msg.data = [-1 for i in range(12)]
            for i in range(6):
                arm_pub_msg.data[i] = int(self.target_joint_pos[i])
                arm_pub_msg.data[i+6] = 1000
            self.arm_pub_.publish(arm_pub_msg)
            self.position_reached = False
            print("published")
        elif not self.position_reached:
            self.timeoutCnt += 1
            if self.timeoutCnt >= 10:
                self.timeoutCnt = 0
                arm_pub_msg = Int16MultiArray()
                arm_pub_msg.data = [-1 for i in range(12)]
                for i in range(6):
                    arm_pub_msg.data[i] = int(self.target_joint_pos[i])
                    arm_pub_msg.data[i+6] = 1000
                self.arm_pub_.publish(arm_pub_msg)
                self.position_reached = False
                print("timeout, published")
        else:
            return


def main():
    rclpy.init()
    node = ArmController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
