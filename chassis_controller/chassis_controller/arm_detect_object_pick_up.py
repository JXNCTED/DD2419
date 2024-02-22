import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from aruco_msgs.msg import MarkerArray
from std_msgs.msg import Int16MultiArray
import numpy as np
import logging
import math
logging.basicConfig(level=logging.ERROR)
logger = logging.getLogger(__name__)


class ArucoDetectPick(Node):
    def __init__(self):
        super().__init__('arm_detect_object_pick_up')
        self.delay_cnt_neutral = 0

        self.neutral_timer = self.create_timer(
            0.01, self.neutral_timer_callback)

        self.arm_pub_ = self.create_publisher(
            Int16MultiArray, '/multi_servo_cmd_sub', 10)

        self.neutral_angles = [12000 for i in range(12)]
        # set the speed of all the controls
        for i in range(6):
            self.neutral_angles[i+6] = 1000

        # HEAD DOWN ASS UP, THAT'S THE WAY WE LIKE TO ...
        self.detect_angles = [12000 for i in range(12)]
        # set the speed of all the controls
        for i in range(6):
            self.detect_angles[i+6] = 1000
        self.detect_angles[2] = 2000
        self.detect_angles[3] = 18000
        self.detect_angles[4] = 10000
        self.detect_angles[0] = 5000

        # Pick up shit...
        self.pickup_angles = [12000 for i in range(12)]
        # set the speed of all the controls
        for i in range(6):
            self.pickup_angles[i+6] = 1000
        self.pickup_angles[2] = 4000
        self.pickup_angles[3] = 15400
        self.pickup_angles[4] = 6160
        self.pickup_angles[0] = 6000

        self.open_gripper = 5000
        self.close_gripper = 12000

        # Count the servo from the bottom to the top

        # common parameter. It refers to link parameter of 4DOF robotic arm
        self.l1 = 0.065  # the distance between robotic arm chassis center and NO.2 servo central shaft is 6.10 cm
        self.l2 = 0.101  # the distance between NO.2 servo and NO.3 servo is 10.16 cm
        self.l3 = 0.094  # the distance between NO.3 servo and NO.4 servo is 9.64cm
        self.l4 = 0.169  # no specific assignment is made here, but reassignment is made based on the selection made during initialization

        self.servo3Range = (0, 1000, 0, 240.0)  # Pulse width， angle
        self.servo4Range = (0, 1000, 0, 240.0)
        self.servo5Range = (0, 1000, 0, 240.0)
        self.servo6Range = (0, 1000, 0, 240.0)

    def theta_1_value(self, degrees):
        return np.max([4000, np.min([20000, int(12000-(degrees-90)*100)])])

    def theta_2_value(self, degrees):
        return np.max([4000, np.min([20000, int(12000+(degrees)*100)])])

    def theta_3_value(self, degrees):
        return np.max([1000, np.min([23000, int(12000-(degrees)*100)])])

    def inverse_kinematicss(self, x, y, l1, l2, l3):
        # Step 1: Find θ1
        theta1 = math.atan2(y - l3, x)

        # Step 2: Find θ2
        r = math.sqrt((x - l1 * math.cos(theta1))**2 +
                      (y - l1 * math.sin(theta1))**2)
        alpha = math.atan2(y - l1 * math.sin(theta1),
                           x - l1 * math.cos(theta1))
        beta = math.acos((l2**2 + r**2 - l3**2) / (2 * l2 * r))
        theta2 = alpha - beta

        # Convert angles to degrees
        theta1_deg = math.degrees(theta1)
        theta2_deg = math.degrees(theta2)

        # θ3 is fixed at 90 degrees
        theta3_deg = 90.0
        print(theta3_deg, theta2_deg, theta1_deg)
        theta3_centideg = 24000-max(0, min(int((theta3_deg+120) * 100), 24000))
        # Map to centidegrees where 0 degrees is 12000
        theta4_centideg = max(0, min(int((theta2_deg + 120) * 100), 24000))
        # Map to centidegrees where 0 degrees is 12000
        theta5_centideg = max(0, min(int((theta1_deg + 120) * 100), 24000))

        return theta3_centideg, theta4_centideg, theta5_centideg

    def new_inverse_kin_from_source(self, coordinate_data, Alpha):

        # Given specific coordinate and pitch angle. Return rotation angle of each joint. If there is no solution, return False.
        # coordinate_data is coordinate of gripper end in cm. It is passed in as tuple, for example (0, 5, 10)
        # Alpha is the angle between gripper and the horizontal plane, in degree.

        # the end of gripper is P (X, Y, Z), coordinate origin is 0, and the origin is the projection of the pan-tilt center on the ground, and the projection of point P on the ground is P_
        # The intersection of l1 and l2 is A, the intersection of l2 and l3 is B, and the intersection of l3 and l4 is C
        # CD is perpendicular to PD, CD is perpendicular to z-axis, then the pitch angle Alpha is the angle between DC and PC, AE is perpendicular to DP_, and E is on DP_, CF is perpendicular to AE, and F is on AE
        # For example, the angle between AB and BC is ABC
        X, Y, Z = coordinate_data
        # Find the rotation angle of the base
        theta6 = math.degrees(math.atan2(Y, X))

        P_O = math.sqrt(X*X + Y*Y)  # distance between P_ and origin 0
        CD = self.l4 * math.cos(math.radians(Alpha))
        # when the pitch angle is positive, PD is positive. When pitch angle is negative, PD is negative
        PD = self.l4 * math.sin(math.radians(Alpha))
        AF = P_O - CD
        CF = Z - self.l1 - PD
        AC = math.sqrt(pow(AF, 2) + pow(CF, 2))
        # if round(CF, 4) < -self.l1:
        #     logger.debug(
        #         'the height is less than 0, CF(%s)<l1(%s)', CF, -self.l1)
        #     return False
        # # The sum of both sides is less than the third side
        # if self.l2 + self.l3 < round(AC, 4):
        #     logger.debug(
        #         'the link structure cannot be built, l2(%s) + l3(%s) < AC(%s)', self.l2, self.l3, AC)
        #     return False

        # Find theat4
        cos_ABC = round(-(pow(AC, 2) - pow(self.l2, 2) -
                        pow(self.l3, 2))/(2*self.l2*self.l3), 4)  # cosine theory
        if abs(cos_ABC) > 1:
            logger.debug(
                'the link structure cannot be built, abs(cos_ABC(%s)) > 1', cos_ABC)
            # cos_ABC = 1
            return False
        ABC = math.acos(cos_ABC)  # inverse trig calculates the radian
        theta4 = 180.0 - math.degrees(ABC)

        # Find theta5
        CAF = math.acos(AF / AC)
        cos_BAC = round((pow(AC, 2) + pow(self.l2, 2) -
                        pow(self.l3, 2))/(2*self.l2*AC), 4)  # cosine theory
        if abs(cos_BAC) > 1:
            logger.debug(
                'the link structure cannot be built, abs(cos_BAC(%s)) > 1', cos_BAC)
            # cos_BAC = 1
            return False
        if CF < 0:
            zf_flag = -1
        else:
            zf_flag = 1
        theta5 = math.degrees(CAF * zf_flag + math.acos(cos_BAC))

        # Find theta3
        theta3 = Alpha - theta5 + theta4

        # When there is solution, return angle dictionary
        self.servo3Range = self.servo3Range
        self.servo4Range = self.servo4Range
        self.servo5Range = self.servo5Range
        self.servo6Range = self.servo6Range
        self.servo3Param = (
            self.servo3Range[1] - self.servo3Range[0]) / (self.servo3Range[3] - self.servo3Range[2])
        self.servo4Param = (
            self.servo4Range[1] - self.servo4Range[0]) / (self.servo4Range[3] - self.servo4Range[2])
        self.servo5Param = (
            self.servo5Range[1] - self.servo5Range[0]) / (self.servo5Range[3] - self.servo5Range[2])
        self.servo6Param = (
            self.servo6Range[1] - self.servo6Range[0]) / (self.servo6Range[3] - self.servo6Range[2])
        print(self.servo3Param, self.servo4Param,
              self.servo5Param, self.servo6Param)

        # Map to centidegrees where 0 degrees is 12000
        theta3_centideg = int((theta3 + 120) * 100)
        # Map to centidegrees where 0 degrees is 12000
        theta4_centideg = int((theta4 + 120) * 100)
        # Map to centidegrees where 0 degrees is 12000
        theta5_centideg = int((theta5 + 120) * 100)
        # Map to centidegrees where 0 degrees is 12000
        theta6_centideg = int((theta6 + 120) * 100)
        print("PRINTING:: --> ", "theta3", theta3, "theta4", theta4,
              "theta5", theta5, "theta6", theta6)
        print("Printing theta5-90 --> ", theta5-90)
        # Map to centidegrees where 0 degrees is 12000
        theta3_centideg = max(0, min(int((theta3 + 120) * 100), 24000))
        # Map to centidegrees where 0 degrees is 12000
        theta4_centideg = max(0, min(int((theta4 + 120) * 100), 24000))
        # Map to centidegrees where 0 degrees is 12000
        theta5_centideg = max(0, min(int((theta5 + 120) * 100), 24000))
        # Map to centidegrees where 0 degrees is 12000
        theta6_centideg = max(0, min(int((theta6 + 120) * 100), 24000))

        return theta3_centideg, theta4_centideg, theta5_centideg, theta6_centideg

    def inverse_kin(self):
        # Length of links in cm
        a1 = 10.1
        a2 = 9.4
        a3 = 16.9

        px = 10  # With this phi conf, it can only handle 18-15cm
        px = -px
        py = -5
        # 18, -14 for pickup

        phi = 4.712  # there are only some valid phi values, check the desmos

        # Equations for Inverse kinematics
        wx = px - a3*np.cos(phi)
        wy = py - a3*np.sin(phi)

        delta = wx**2 + wy**2
        c2 = (delta - a1**2 - a2**2)/(2*a1*a2)

        s2 = np.sqrt(1-c2**2)  # elbow down
        theta_2 = np.arctan2(s2, c2)

        s1 = ((a1+a2*c2)*wy - a2*s2*wx)/delta
        c1 = ((a1+a2*c2)*wx + a2*s2*wy)/delta
        theta_1 = np.arctan2(s1, c1)
        theta_3 = phi-theta_1-theta_2

        print('theta_1: ', np.rad2deg(theta_1))
        print('theta_2: ', np.rad2deg(theta_2))
        print('theta_3: ', np.rad2deg(theta_3))

        return np.rad2deg(theta_1), np.rad2deg(theta_2), np.rad2deg(theta_3)

    def neutral_timer_callback(self):
        # Set the arm to the new neutral. "LOOKING DOWN all the time"
        self.get_logger().info(f"{self.delay_cnt_neutral}")
        self.delay_cnt_neutral += 1
        if self.delay_cnt_neutral < 750:
            arm_cmd = Int16MultiArray()
            arm_cmd.data = [12000 for i in range(12)]

            theta_1, theta_2, theta_3 = self.inverse_kin()

            theta_1_16bit = self.theta_1_value(theta_1)
            theta_2_16bit = self.theta_2_value(theta_2)
            theta_3_16bit = self.theta_3_value(theta_3)
            theta_1_16bit = 12000
            theta_2_16bit = 12000
            theta_3_16bit = 12000

            print(
                f"servo1: {theta_1_16bit}, servo2: {theta_2_16bit}, servo3: {theta_3_16bit}")

            # ID on this servo is 5 THETA_1 4000IS ABOUT 0 DEGREES.
            # ADD 90 degrees here!!! Important since our robot is angleddetect_angles 90 degrees more than the youtube/github solution.
            #
            # arm_cmd.data[4] = self.theta_1_value(-67+90)
            bottom_servo = arm_cmd.data[4]
            arm_cmd.data[4] = theta_1_16bit

            """For the servo at the bottom. The least theta_1 is with a value of 4000. 90 is 12000 and 180 is 20000"""
            """Middle servo, theta_2, the bottom 0deg is with 21000 and the maximum theta=180 is with 3000 """
            """Top servo. theta_3. 0deg= 3000, 180 deg= 20000"""
            # ID on servo is 4 THETA_2
            arm_cmd.data[3] = theta_2_16bit

            # Two is actually three (going by Id on the servos)
            arm_cmd.data[2] = theta_3_16bit  # THETA_3
            # set the speed of all the controls
            for i in range(6):
                arm_cmd.data[i+6] = 1000

            # arm_cmd.data[0] = self.open_gripper
            # arm_cmd.data[4] = 12000
            # arm_cmd.data[3] = 12000
            # arm_cmd.data[2] = 12000
            # self.theta33, self.theta44, self.theta55 = self.inverse_kinematicss(
            #     0.03, 0.06, self.l1, self.l2, self.l3)

            # print(self.theta33)

            # set_theta3, set_theta4, set_theta5, set_theta6 = self.new_inverse_kin_from_source(
            #     [0, 0.1, 1*(self.l1 + self.l2 + self.l3 + self.l4)], 90)

            # print(set_theta3, set_theta4, set_theta5, set_theta6)
            # arm_cmd.data[2] = self.theta33
            # arm_cmd.data[3] = self.theta44
            # arm_cmd.data[4] = self.theta55
            # arm_cmd.data[5] = set_theta6
            # arm_cmd.data[5] = 15000            #ROTATION OF THE BASE

            # PREDEFINED POSITION FOR THE ARM
            # arm_cmd.data = self.neutral_angles  # STRAIGHT UP
            # arm_cmd.data = self.detect_angles  # DETECTING MODE, SEARCHING
            arm_cmd.data = self.pickup_angles  # PICKING UP
            self.arm_pub_.publish(arm_cmd)
        else:
            self.has_set_neutral = True
            self.neutral_timer.cancel()

    # Everything it can see it can reach.
    # Use the /image_raw


def main():
    rclpy.init()
    node = ArucoDetectPick()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
