# Hi Sean!
# I have created this node in order to compare the current servo values with the desired values
# It creates a subscriber to the servo_pos_publisher and multi_servo_cmd_sub
# When these do not differ more than 10% on the 6 joints, they are thought to be the same
# If not, then they are not the same.
# This seems to work fine. Just publish a joint value desired to the topic and then see the node comparing the desired
# To the actual value right now.

import random
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

        # Create a timer that calls the timer_callback function every timer_interval seconds
        self.timer = self.create_timer(1, self.timer_callback)
        self.timer.cancel()

        # Flag to track the timer state
        self.is_timer_running = False

        self.i = 0

    def timer_callback(self):
        self.timer.cancel()
        self.is_timer_running = False
        self.get_logger().info("timer stopped")

    # When the servo_sub message and the arm_sub are approximately the same, then we can publish the next message.

    def check_set_servo(self, msg: Int16MultiArray):
        # These are the value that we want the servos to be!
        self.set_servo_msg = msg.data
        print("CHECK_SET", msg)

        self.is_timer_running = True
        self.timer.reset()
        # self.check_and_publish()

    def check_curr_servo(self, msg: JointState):
        self.curr_servo_msg = msg.position
        # print("CHECK CURR", msg)

    def check_and_publish(self):
        if self.curr_and_set_values_the_same():
            print("The messages are approximately the same")

    # Checks if the values are approx the same :)
    def curr_and_set_values_the_same(self):
        break_early = False
        if (self.set_servo_msg and self.curr_servo_msg):
            # Loop through the interesting joints
            for i in range(4, 6):
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
                randNum = random.randint(0, 100)
                print("NOT THE SAME", randNum)
                return False

    def publish_to_arm_conf(self):
        while self.i < 10000:
            self.i += 1
            print(self.i)
            if not self.is_timer_running:
                # Implement logic to publish to /arm_conf
                # You may also set self.has_finished to True to prevent repeated publishing
                msg = String()
                msg.data = random.choice(
                    ["detect", "neutral", "open", "close", "pickup"])
                self.get_logger.info(f"{msg.data}")
                self.arm_conf_pub.publish(msg)


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

# Realsense detect and move to object
# when distance to object is at 0.25, detect using arm camera - boolean topic + help us Ed
# find the middle point and move to make it in the middle
# when this happens, then pickup and close
# then neutral
