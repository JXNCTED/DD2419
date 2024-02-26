# Hi Sean!
# I have created this node in order to compare the current servo values with the desired values
# It creates a subscriber to the servo_pos_publisher and multi_servo_cmd_sub
# When these do not differ more than 10% on the 6 joints, they are thought to be the same
# If not, then they are not the same.
# This seems to work fine. Just publish a joint value desired to the topic and then see the node comparing the desired
# To the actual value right now.

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int16MultiArray, String
from sensor_msgs.msg import JointState

import logging
logging.basicConfig(level=logging.ERROR)
logger = logging.getLogger(__name__)


class ArmListen(Node):
    def __init__(self):
        super().__init__('ArmListen')

        # ------------------------------PUBS AND SUBS---------------------------------

        # Subscribe to the multi servo cmd sub (Tells us the published servo values)
        self.servo_sub = self.create_subscription(
            Int16MultiArray, '/multi_servo_cmd_sub', self.check_set_servo, 10)

        # Subscribe to the servo_pos_publisher (Tells us current servo values)
        self.arm_sub = self.create_subscription(
            JointState, '/servo_pos_publisher', self.check_curr_servo, 10)  # Type JointState gives current servo readings

        # String configuration
        self.arm_conf_pub = self.create_publisher(
            String, '/arm_conf', 10
        )

        # ----------------------------VARIABLES----------------------------------------
        self.has_finished = False
        self.curr_servo_msg = None
        self.set_servo_msg = None

    # When the servo_sub message and the arm_sub are approximately the same, then we can publish the next message.

    def check_set_servo(self, msg: Int16MultiArray):
        # These are the value that we want the servos to be!
        # print("CHECK_SET", msg.data)
        self.set_servo_msg = msg.data
        self.check_and_publish()

    def check_curr_servo(self, msg: JointState):
        # print("CHECK_CURR")
        # print("CHECK CURR", msg.position)
        self.curr_servo_msg = msg.position
        self.check_and_publish()

    def check_and_publish(self):
        if self.curr_and_set_values_the_same():
            print("The messages are approximately the same")

        pass
        # if self.prev_servo_msg is not None and self.curr_servo_msg is not None:
        #     # Implement logic to check if the messages are approximately the same
        #     if self.are_messages_approximately_same():
        #         # Here check the current value of the middle of the cube and set the robot state to correct one.
        #         self.publish_to_arm_conf()

    # Checks if the values are approx the same :)
    def curr_and_set_values_the_same(self):
        break_early = False
        if (self.set_servo_msg and self.curr_servo_msg):
            # Loop through the interesting joints
            for i in range(0, 6):
                # If it differs by more than 10%  ???
                frac = None
                if (self.curr_servo_msg[i] > self.set_servo_msg[i]):
                    frac = (
                        self.curr_servo_msg[i]-self.set_servo_msg[i])/self.curr_servo_msg[i]
                else:
                    frac = (
                        self.set_servo_msg[i]-self.curr_servo_msg[i])/self.set_servo_msg[i]
                if (frac >= 0.1):
                    break_early = True

            if (not break_early):
                print("THE SAME")
                return True
            else:
                print("NOT THE SAME")
                return False

    def publish_to_arm_conf(self):
        if self.has_finished:
            # Implement logic to publish to /arm_conf
            # You may also set self.has_finished to True to prevent repeated publishing
            pass


def main():
    rclpy.init()
    node = ArmListen()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
