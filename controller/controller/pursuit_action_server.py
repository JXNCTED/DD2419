import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from robp_interfaces.action import Pursuit


class PursuitActionServer(Node):
    def __init__(self):
        super().__init__("pursuit_action_server")

        self.odom_x = 0
        self.odom_y = 0
        self.odom_yaw = 0
        self.waypoints = None

        self.action_server = ActionServer(
            self, Pursuit, "pursuit", self.execute_callback
        )
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )
        self.publish_vel = self.create_publisher(
            Twist,
            '/motor_controller/twist',
            10
        )

    # Pseudo code sort of
    def execute_callback(self, goal_handle):
        self.get_logger().info("goal received")
        self.waypoints = goal_handle.request.waypoints
        self.get_logger().info(f"waypoints number: {len(self.waypoints)}")
        self.get_logger().info(
            f"start with {self.waypoints[0]}, end with {self.waypoints[-1]}")
        goal_handle.succeed()
        result = Pursuit.Result()
        return result

        # feedback_msg = Pursuit.Feedback()
        # twist = Twist()
        # LOOK_AHEAD = 0.5
        # for idx, waypoint in enumerate(goal_handle.request.waypoints):
        #     self.get_logger().info("Moving towards waypoint:" + str(idx))
        #     lin, ang, t = self.velocity(waypoint)
        #     feedback_msg.current_velocity.linear.x = lin
        #     feedback_msg.current_velocity.angular.z = ang
        #     self.publish_vel.publish(feedback_msg.current_velocity)
        #     goal_handle.publish_feedback(feedback_msg)
        # time.sleep(t)
        # while odom.x and y is not close to waypoint stall
        # twist.linear.x = 0.0
        # twist.angular.z = 0.0
        # self.publish_vel.publish(twist)
        # self.get_logger().info("Done")
        # goal_handle.succeed()
        # result = Pursuit.Result()
        # result.success = True
        # return result

    # Not implemented yet see test_pure_pursuit in robotics_group7

    def loop(self):
        while (rclpy.ok()):
            rclpy.spin_once(self)
            if not self.waypoints or len(self.waypoints) == 0:
                # self.get_logger().info("No waypoints")
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.publish_vel.publish(twist)
                continue
            if np.hypot(self.waypoints[-1].x - self.odom_x, self.waypoints[-1].y - self.odom_y) < 0.1:
                self.get_logger().info("Reached waypoint")
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.publish_vel.publish(twist)
                self.waypoints = None
                continue
            for idx, waypoint in enumerate(self.waypoints):
                LOOK_AHEAD = 0.2
                if np.hypot(waypoint.x - self.odom_x, waypoint.y - self.odom_y) > LOOK_AHEAD:
                    break
                self.waypoints.pop(0)
            if len(self.waypoints) == 0:
                continue
            lin, ang, t = self.velocity(self.waypoints[0])
            twist = Twist()
            twist.linear.x = lin
            twist.angular.z = ang
            self.publish_vel.publish(twist)

    def velocity(self, target):
        x = self.odom_x
        y = self.odom_y
        yaw = self.odom_yaw
        dx = target.x - x
        dy = target.y - y
        y_cos = np.cos(yaw)
        y_sin = np.sin(yaw)
        tx = dx * y_cos + dy * y_sin
        ty = -dx * y_sin + dy * y_cos
        dist = np.hypot(tx, ty)
        alpha = np.arctan2(ty, tx)
        radius = dist / (2 * np.sin(alpha))
        lin_v = 0.5
        arc_length = 2.0 * alpha * radius
        time = arc_length / lin_v
        if radius < 1000:
            ang_v = lin_v / radius
        else:
            ang_v = 0.0
        return lin_v, ang_v, time

    def odom_callback(self, msg):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        self.odom_yaw = np.arctan2(
            2.0 * (w * z + x * y), w * w + x * x - y * y - z * z)


def main(args=None):
    rclpy.init(args=args)

    pursuit_action_server = PursuitActionServer()

    # rclpy.spin(pursuit_action_server)
    pursuit_action_server.loop()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
