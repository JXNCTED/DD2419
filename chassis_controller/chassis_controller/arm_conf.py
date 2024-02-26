import rclpy
from rclpy.node import Node

from std_msgs.msg import Int16MultiArray, String
import numpy as np

import logging
logging.basicConfig(level=logging.ERROR)
logger = logging.getLogger(__name__)


class ArmConf(Node):
    """
    Move arm into predefined positions depending on the incoming message.
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

        # timer for sending messages
        self.delay_cnt_neutral = 0
        # whether we have finished sending the current command
        self.has_finished = False

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
        self.pickup_angles[3] = 15400
        self.pickup_angles[4] = 6160
        self.pickup_angles[0] = self.open_gripper

        # dict for these
        self.arm_dict = {'pickup': self.pickup_angles, 'detect': self.detect_angles,
                         'neutral': self.neutral_angles, 'open': self.open_gripper, 'closed': self.close_gripper}
        # previous incoming message
        self.prev_msg = None

    def conf_callback(self, msg: String):
        # Set the arm to the new neutral. "LOOKING DOWN all the time"
        self.get_logger().info(f"{self.delay_cnt_neutral}")
        self.delay_cnt_neutral += 1
        if self.delay_cnt_neutral < 750:
            self.has_finished = False

            arm_cmd = Int16MultiArray()

            if self.prev_msg != msg:
                self.prev_msg = msg

                # PREDEFINED POSITION FOR THE ARM
                arm_cmd.data = self.arm_dict[msg]
                self.arm_pub_.publish(arm_cmd)
        else:
            self.has_finished = True


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
