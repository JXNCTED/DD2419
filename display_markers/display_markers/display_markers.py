#!/usr/bin/env python

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler, euler_from_quaternion, quaternion_about_axis, quaternion_multiply
import math


import tf2_geometry_msgs

from aruco_msgs.msg import MarkerArray
from geometry_msgs.msg import TransformStamped


class DisplayMarkers(Node):

    def __init__(self):

        super().__init__('display_markers')

        # Initialize the transform listerner and assign it a buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Initialize the transform broadcaster
        self._tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to aruco marker topic and call callback function on each recieved message
        self.subscription = self.create_subscription(
            MarkerArray,
            '/marker_publisher/markers',
            self.aruco_callback,
            10
        )
        self.subscription

    def aruco_callback(self, msg: MarkerArray):
        t = TransformStamped()
        for marker in msg.markers:

            t.header.stamp = marker.header.stamp

            t.child_frame_id = f'/aruco/detected{marker.id}'
            t.header.frame_id = 'odom'

            # Looks up transform between map and baselink.
            try:
                look_trans = self.tf_buffer.lookup_transform(
                    "odom", "camera_color_optical_frame", msg.header.stamp)
            except Exception as e:
                # print("Failed transformation")
                self.get_logger().warn(str(e))
                continue

            # Applies the transformation.
            marker_transformed = tf2_geometry_msgs.do_transform_pose(
                marker.pose.pose, look_trans)

            # Updates transformstamped message with the marker_transformed.
            t.transform.translation.x = marker_transformed.position.x
            t.transform.translation.y = marker_transformed.position.y
            t.transform.translation.z = marker_transformed.position.z

            t.transform.rotation.x = marker_transformed.orientation.x
            t.transform.rotation.y = marker_transformed.orientation.y
            t.transform.rotation.z = marker_transformed.orientation.z
            t.transform.rotation.w = marker_transformed.orientation.w

            # Publish the message.
            try:
                self._tf_broadcaster.sendTransform(t)
            except TransformException:
                print("Error transforming")


def main():
    rclpy.init()
    node = DisplayMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
