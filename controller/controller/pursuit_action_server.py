import numpy as np
import rclpy
import rclpy.duration
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose
import rclpy.time
from robp_interfaces.action import Pursuit
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose, TransformStamped


class PursuitActionServer(Node):
    def __init__(self):
        super().__init__('pursuit_action_server')

        self.odom_x = 0
        self.odom_y = 0
        self.odom_yaw = 0
        self._cb_group = ReentrantCallbackGroup()
        self.waypoints = []
        self.rate = self.create_rate(100, self.get_clock())

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer, self, spin_thread=True)

        self.odom_to_map_tf = TransformStamped()

        self.running = False

        self._action_server = ActionServer(
            self,
            Pursuit,
            'pursuit',
            execute_callback=self.execute_callback,
            callback_group=self._cb_group,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        self._odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            callback=self.odom_callback,
            qos_profile=10,
            callback_group=self._cb_group
        )
        self._publish_vel = self.create_publisher(
            Twist,
            '/motor_controller/twist',
            qos_profile=10,
            callback_group=self._cb_group
        )

    def destroy(self):
        """Cleanup"""
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        """Accepts goal if there are waypoints, otherwise rejects goal"""
        waypoints = goal_request.waypoints
        self.get_logger().info('Received goal request')
        if waypoints is None or len(waypoints) == 0:
            self.get_logger().info('Rejecting the goal request')
            return GoalResponse.REJECT
        else:
            self.waypoints = waypoints
        self.get_logger().info('Accepting the goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, cancel_request):
        """Accepts the cancel requests"""
        self.get_logger().info('Accepting cancel request')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """Execute the pursuit goal"""
        self.get_logger().info('Executing pursuit goal...')
        result = Pursuit.Result()
        result.success = False

        self.running = True

        goal_point = self.waypoints[-1]
        twist = Twist()
        # align the robot with start point
        angle = np.arctan2(goal_point.y - self.odom_y,
                           goal_point.x - self.odom_x)
        while abs(angle - self.odom_yaw) > 0.1:
            self.rate.sleep()
            angle = np.arctan2(goal_point.y - self.odom_y,
                               goal_point.x - self.odom_x)
            # self.get_logger().info(f"Aligning with start point: {angle - self.odom_yaw}")
            twist.linear.x = 0.0
            twist.angular.z = 0.4 * np.sign(angle - self.odom_yaw)
            self._publish_vel.publish(twist)

        while (len(self.waypoints) > 0):
            if goal_handle.is_cancel_requested:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self._publish_vel.publish(twist)
                goal_handle.canceled()
                self.get_logger('Goal canceled')
                self.running = False
                return result
            if np.hypot(self.waypoints[-1].x - self.odom_x, self.waypoints[-1].y - self.odom_y) < 0.19:
                self.waypoints = []
                result.success = True
                break
            for waypoint in self.waypoints:
                # TODO: make this a parameter to to set
                LOOK_AHEAD = 0.10
                # if current waypoint is beyond LOOK_AHEAD, use it as waypoint
                # I imagine this could cause a problem if the last waypoint is
                # beyond 0.1 but within 0.2 could check for this case and move
                # based on time to the goal point.
                if np.hypot(waypoint.x - self.odom_x, waypoint.y - self.odom_y) > LOOK_AHEAD:
                    break

                self.waypoints.pop(0)
            if len(self.waypoints) == 0:
                break

            lin, ang, t = self.velocity(self.waypoints[0])
            twist = Twist()
            twist.linear.x = lin
            twist.angular.z = ang
            self._publish_vel.publish(twist)
            self.rate.sleep()

        twist = Twist()

        # align with the goal point
        self.rate.sleep()

        angle = np.arctan2(goal_point.y - self.odom_y,
                           goal_point.x - self.odom_x)


        self.get_logger().info(
            f"Aligning with goal point: {angle - self.odom_yaw}")
        while abs(angle - self.odom_yaw) > 0.08: # 5 degress?
            self.get_logger().info(
                f"Aligning with goal point: {angle - self.odom_yaw}")
            self.rate.sleep()
            angle = np.arctan2(goal_point.y - self.odom_y,
                               goal_point.x - self.odom_x)
            twist.linear.x = 0.0
            twist.angular.z = 0.4 * np.sign(angle - self.odom_yaw)
            self._publish_vel.publish(twist)

        # stop the robot
        for _ in range(10):
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self._publish_vel.publish(twist)
            self.rate.sleep()

        if result.success:
            self.get_logger().info('\033[92mGoal succeeded\033[0m')
            goal_handle.succeed()
        else:
            self.get_logger().warn('Goal not succeeded, aborting...')
            goal_handle.abort()

        self.running = False
        return result

    def velocity(self, target, lin_v=0.1):
        """
        Calculate the angular velocity from the constant linear velocity,
        the position of the robot and the waypoint using a circle.
        """
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
        if abs(np.sin(alpha)) < 0.01:
            radius = 10000.0
            ang_v = 0.0
            arc_length = dist
        else:
            radius = dist / (2 * np.sin(alpha))
            arc_length = 2.0 * alpha * radius
            ang_v = lin_v / radius
        t = arc_length / lin_v  # time to move to the waypoint, not used atm
        return lin_v, ang_v, t

    def odom_callback(self, msg: Odometry):
        """
        Extract the x, y and yaw from the odometry.
        https://robotics.stackexchange.com/questions/16471/get-yaw-from-quaternion
        """
        if not self.running:
            return
        # x = msg.pose.pose.orientation.x
        # y = msg.pose.pose.orientation.y
        # z = msg.pose.pose.orientation.z
        # w = msg.pose.pose.orientation.w
        # self.odom_yaw = np.arctan2(
        #     2.0 * (w * z + x * y), w * w + x * x - y * y - z * z)

        # for simplicty, let the arm center for approach center
        CAMERA_ARM_OFFSET = 0.055
        try:
            self.odom_to_map_tf = self.tf_buffer.lookup_transform(
                'map', 'odom', rclpy.time.Time().to_msg(), timeout=rclpy.duration.Duration(seconds=1.0))
        except Exception as e:
            self.get_logger().error(
                f"Failed to lookup transform: {str(e)}")
            # return

        odom_pose = msg.pose.pose
        map_pose: Pose
        map_pose = do_transform_pose(
            odom_pose, self.odom_to_map_tf)

        x = map_pose.orientation.x
        y = map_pose.orientation.y
        z = map_pose.orientation.z
        w = map_pose.orientation.w
        self.odom_yaw = np.arctan2(
            2.0 * (w * z + x * y), w * w + x * x - y * y - z * z)

        self.odom_x = map_pose.position.x + CAMERA_ARM_OFFSET * np.sin(self.odom_yaw)
        self.odom_y = map_pose.position.y + CAMERA_ARM_OFFSET * np.cos(self.odom_yaw)


def main(args=None):
    rclpy.init(args=args)

    executor = rclpy.executors.MultiThreadedExecutor()
    pursuit_action_server = PursuitActionServer()

    rclpy.spin(pursuit_action_server, executor=executor)

    pursuit_action_server.destroy()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
