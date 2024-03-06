import rclpy
from rclpy.node import Node

from std_msgs.msg import Int16MultiArray, String

import logging
logging.basicConfig(level=logging.ERROR)
logger = logging.getLogger(__name__)


class ArmConf(Node):
    """
    Move arm into predefined positions depending on the incoming message.
    We have to explicitly tell this node what state we want it to be in, \
    we must tell it if we want it to be neutral, detect, or pickup, and then specifically \
    if it is open or closed.
    """

    def __init__(self):
        super().__init__('arm_conf')

        # publish servo values
        self.arm_pub_ = self.create_publisher(
            Int16MultiArray, '/multi_servo_cmd_sub', 10)

        # String configuration
        self.arm_conf_sub_ = self.create_subscription(
            String, '/arm_conf', self.conf_callback, 10
        )

        # gripper configurations
        self.open_gripper = 5000
        self.close_gripper = 12000

        # Neutral phase
        self.neutral_angles = [12000 for i in range(12)]
        # set the speed of all the controls
        for i in range(6):
            self.neutral_angles[i+6] = 1000
        # set gripper
        self.neutral_angles[0] = self.open_gripper

        # Detect phase
        self.detect_angles = [12000 for i in range(12)]
        # set the speed of all the controls
        for i in range(6):
            self.detect_angles[i+6] = 1000
        # set conf
        self.detect_angles[2] = 2000
        self.detect_angles[3] = 18000
        self.detect_angles[4] = 10000
        self.detect_angles[0] = self.open_gripper

        # Pickup phase
        self.pickup_angles = [12000 for i in range(12)]
        # set the speed of all the controls
        for i in range(6):
            self.pickup_angles[i+6] = 1000
        # set conf
        self.pickup_angles[2] = 4000
        self.pickup_angles[3] = 15500
        self.pickup_angles[4] = 5300
        self.pickup_angles[0] = self.open_gripper

        # dict for these
        self.arm_dict = {'pickup': self.pickup_angles, 'detect': self.detect_angles,
                         'neutral': self.neutral_angles, 'open': self.open_gripper, 'close': self.close_gripper}
        # previous incoming messages
        # always start with open and neutral
        self.prev_msg = String()
        self.prev_msg.data = 'neutral'
        self.prev_grip_msg = String()
        self.prev_grip_msg.data = 'open'

    def conf_callback(self, msg: String):
        # dont do anything if we are already in this state
        if self.prev_msg == msg or self.prev_grip_msg == msg:
            return

        arm_cmd = Int16MultiArray()

        if msg.data == 'open' or msg.data == 'close':
            # set to previous message but then set the gripper accordingly
            prev = self.arm_dict[str(self.prev_msg.data)]
            prev[0] = self.arm_dict[str(msg.data)]
            # use hardcoded positioning
            arm_cmd.data = prev
            # update previous gripper
            self.prev_grip_msg = msg
        else:
            # update
            self.prev_msg = msg
            # use hardcoded positioning
            arm_cmd.data = self.arm_dict[str(msg.data)]
            # grip will always be set to what it last was
            arm_cmd.data[0] = self.arm_dict[str(self.prev_grip_msg.data)]

        # publish
        self.arm_pub_.publish(arm_cmd)


def main():
    rclpy.init()
    node = ArmConf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
