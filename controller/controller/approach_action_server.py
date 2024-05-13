
import numpy as np
import rclpy
import rclpy.logging
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import Twist, Transform, Pose
import rclpy.time
from tf2_geometry_msgs import do_transform_pose
from robp_interfaces.action import Approach
from aruco_msgs.msg import MarkerArray
from tf2_ros import Buffer, TransformListener

from threading import Lock


from detection_interfaces.msg import DetectedObj, BoxList, Box


def velocity(pose):
    x = pose.position.z
    y = -pose.position.x
    theta = np.arctan2(x, y)
    DISP = 0.15
    CAMERA_OFFSET = 0.05

    # make the robot goto 15cm in front of the marker
    x, y = x-np.sin(theta)*DISP, y-np.cos(theta)*DISP
    y = y + CAMERA_OFFSET
    theta = np.arctan2(y, x)

    v = 0.1
    d = np.sqrt(x**2 + y**2)
    R = d / (2 * np.sin(theta / 2))
    l = R * theta
    t = l / v
    w = theta / t

    # rclpy.logging.get_logger('approach_action_server').info(
    #     f'x: {x}, y: {y}x theta: {theta}, v: {v}, w: {w}, t: {t}')

    return v, w, t


class ApproachActionServer(Node):
    def __init__(self):
        super().__init__('approach_action_server')

        self.target = None

        self._cb_group = ReentrantCallbackGroup()

        self.objects = []
        self.markers = []
        self.box_list = [[0, 0] for _ in range(3)]
        self.rate = self.create_rate(100, self.get_clock())
        self.aruco_stamp = rclpy.time.Time()
        self.object_stamp = rclpy.time.Time()

        self.marker_in_view = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer, self, spin_thread=True)

        # all possible targets
        self.ACCEPTABLE_TARGETS = [
            str(i) for i in range(1, 15)]
        # all possible aruco markers
        # self.ACCEPTABLE_ARUCOS.extend([f'aruco_{i}' for i in range(1, 4)])
        self.ACCEPTABLE_ARUCOS = [f'aruco_{i}' for i in range(1, 4)]
        self.get_logger().info(
            f'Acceptable targets: {self.ACCEPTABLE_TARGETS, self.ACCEPTABLE_ARUCOS}')

        self._action_server = ActionServer(
            self,
            Approach,
            'approach',
            execute_callback=self.execute_callback,
            callback_group=self._cb_group,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self._publish_vel = self.create_publisher(
            Twist,
            '/motor_controller/twist',
            qos_profile=10,
            callback_group=self._cb_group
        )

        self._detected_pos_sub = self.create_subscription(
            DetectedObj,
            '/detection_ml/detected_obj',
            self.detected_pos_callback,
            qos_profile=10,
            callback_group=self._cb_group
        )

        self.aruco_sub = self.create_subscription(
            MarkerArray,
            'marker_publisher/markers',
            self.aruco_callback,
            qos_profile=10,
            callback_group=self._cb_group
        )

        self.box_sub = self.create_subscription(
            BoxList, '/box_list', self.box_callback, qos_profile=10, callback_group=self._cb_group)

    def aruco_callback(self, msg: MarkerArray):
        self.aruco_stamp = msg.header.stamp
        self.markers.clear()
        for marker in msg.markers:
            self.markers.append((marker.pose.pose, marker.id))

        if self.target is None:
            return
        else:
            self.marker_in_view = int(self.target.split('_')[1]) in [
                id for (_, id) in self.markers]

    def box_callback(self, msg: BoxList):
        box: Box
        for i, box in enumerate(msg.boxes):
            index = box.aruco_id - 1
            self.box_list[index] = [
                box.center_pose.position.x, box.center_pose.position.y]

    def detected_pos_callback(self, msg: DetectedObj):
        self.object_stamp = msg.header.stamp
        self.objects.clear()
        for obj in msg.obj:
            self.objects.append(
                (obj.position, obj.category))

    def destroy(self):
        self.get_logger().info('Destroying...')
        self._action_server.destroy()
        super().destroy_node()

    def execute_callback(self, goal_handle):
        """Execute callback for the action server"""
        self.get_logger().info('Executing goal...')
        result = Approach.Result()

        result.success = False
        self.marker_in_view = False

        if self.target is None:
            self.get_logger().warn(
                "Target not set, aborting goal.")
            goal_handle.abort()
            self.target = None
            return result

        if self.target in self.ACCEPTABLE_ARUCOS:
            # first align with the marker
            twist = Twist()

            while True:
                tf_map_base = self.tf_buffer.lookup_transform(
                    'base_link', 'map', rclpy.time.Time())

                # qw = tf_map_base.transform.rotation.w
                # qx = tf_map_base.transform.rotation.x
                # qy = tf_map_base.transform.rotation.y
                # qz = tf_map_base.transform.rotation.z
                # # wrap 0 to 2pi
                # yaw = np.arctan2(2.0 * (qw*qz + qx*qy),
                #                     1.0 - 2.0 * (qy*qy + qz*qz))

                pose_box_in_map = Pose()
                pose_box_in_map.position.x = self.box_list[int(
                    self.target.split('_')[1]) - 1][0]
                pose_box_in_map.position.y = self.box_list[int(
                    self.target.split('_')[1]) - 1][1]

                pose_box_in_base = do_transform_pose(
                    pose_box_in_map, tf_map_base)

                # dx = pose_box_in_base.pose.position.x
                dx = pose_box_in_base.position.x
                dy = pose_box_in_base.position.y

                angle = np.arctan2(dy, dx)

                # box_pose = self.box_list[int(self.target.split('_')[1]) - 1]
                # dx = box_pose[0] - tf_map_base.transform.translation.x
                # dy = box_pose[1] - tf_map_base.transform.translation.y

                self.get_logger().info(
                    f'arget: {np.arctan2(dx, dy)}')

                # break if the angle is small enough or the target is in the view
                if abs(angle) < 0.1 or self.marker_in_view:
                    break

                twist.linear.x = 0.0
                # twist.angular.z = -0.2 if angle > 0 else 0.2
                twist.angular.z = 0.2 * np.sign(angle)

                self._publish_vel.publish(twist)

            for _ in range(100):
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self._publish_vel.publish(twist)
                self.rate.sleep()

            pose = None
            for (pose_det, id) in self.markers:
                self.get_logger().info(f'id: {id}')
                delta = self.get_clock().now().nanoseconds - \
                    rclpy.time.Time().from_msg(self.aruco_stamp).nanoseconds
                self.get_logger().info(f'delta: {delta}')
                if f"aruco_{id}" == self.target and delta < 3e9:
                    pose = pose_det
                    break

            if pose is None:
                self.get_logger().warn(
                    f" Target {self.target} not found, aborting goal.")
                goal_handle.abort()
                self.target = None
                return result

            # while (pose.position.z > 0.25):
            #     self.get_logger().info(f'distance: {pose.position.z}')
            #     twist.linear.x, twist.angular.z, t = velocity(pose)
            #     self._publish_vel.publish(twist)
            #     cnt = 0
            #     pose = None
            #     while (cnt < 35):
            #         for (pose_det, id) in self.markers:
            #             delta = self.get_clock().now().nanoseconds - \
            #                 rclpy.time.Time().from_msg(self.aruco_stamp).nanoseconds
            #             if f"aruco_{id}" == self.target and delta < 3e9:
            #                 pose = pose_det
            #                 break
            #         if pose is not None:
            #             break
            #         self.rate.sleep()
            #         cnt += 1
            #     if pose is None:
            #         break

            twist.linear.x, twist.angular.z, t = velocity(pose)
            self.get_logger().info(
                f'v: {twist.linear.x}, w: {twist.angular.z}, t: {t}')
            rate = self.create_rate(1.0/t, self.get_clock())
            self._publish_vel.publish(twist)
            rate.sleep()

            for _ in range(100):
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self._publish_vel.publish(twist)
                self.rate.sleep()

            # if pose is None:
            #     self.get_logger().warn("timeout")
            #     goal_handle.abort()
            #     result.success = False
            #     return result

        elif self.target in self.ACCEPTABLE_TARGETS:
            point = None

            # find the target from objects
            cnt = 0
            TIMEOUT = 75
            while (cnt < TIMEOUT):
                for (point_det, category) in self.objects:
                    if str(category) == self.target:
                        point = point_det
                        break
                if point is not None:
                    break

                self.rate.sleep()
                cnt += 1

            if point is None:
                self.get_logger().warn(
                    f" Target {self.target} not found, aborting goal.")
                result.success = False
                self.target = None
                goal_handle.abort()
                return result

            # goto pose
            twist = Twist()
            KP = -1.0
            while (point.point.z > 0.28):
                self.get_logger().info(f'distance: {point.point.z}')
                twist.angular.z = point.point.x * KP
                twist.linear.x = 0.2
                self._publish_vel.publish(twist)
                cnt = 0
                point = None
                while (cnt < TIMEOUT):
                    for (point_det, category) in self.objects:
                        # if str(category) == self.target and rclpy.time.Time().nanoseconds - self.object_stamp.nanoseconds < 5e8:
                        if str(category) == self.target and rclpy.time.Time().nanoseconds - rclpy.time.Time().from_msg(self.object_stamp).nanoseconds < 3e9:
                            point = point_det
                            break
                    if point is not None:
                        break
                    self.rate.sleep()
                    cnt += 1
                if point is None:
                    break

                self.rate.sleep()

            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self._publish_vel.publish(twist)
            if point is None:
                self.get_logger().warn("timeout")

        for _ in range(50):
            twist.linear.x = 0.0
            self._publish_vel.publish(twist)
            self.rate.sleep()

        self.get_logger().info('\033[92mGoal succeeded\033[0m')

        goal_handle.succeed()
        result.success = True
        self.objects.clear()

        return result

    def goal_callback(self, goal_request):
        self.get_logger().info(f'Received goal request: {goal_request.target}')
        target = goal_request.target

        if target not in self.ACCEPTABLE_TARGETS and target not in self.ACCEPTABLE_ARUCOS:
            self.get_logger().info(
                f'rejecting {target}, not in {self.ACCEPTABLE_TARGETS, self.ACCEPTABLE_ARUCOS}')
            return GoalResponse.REJECT

        self.target = target
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Cancel callback for the action server"""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT


def main():
    rclpy.init()
    node = ApproachActionServer()
    executor = rclpy.executors.MultiThreadedExecutor(3)
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    node.destroy()
    rclpy.shutdown()
