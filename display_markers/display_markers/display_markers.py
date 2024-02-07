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
            t.header.frame_id = "map"

            # Looks up transform between map and baselink.
            try:
                look_trans = self.tf_buffer.lookup_transform(
                    "map", "base_link", marker.header.stamp)
            except:
                print("Failed transformation")
                return

            # Save position in temp before offsetting.
            temp_x_pos = marker.pose.pose.position.x
            temp_y_pos = marker.pose.pose.position.y
            temp_z_pos = marker.pose.pose.position.z

            # Adjust position according to offsets camera and robot.
            marker.pose.pose.position.x = temp_z_pos + 0.08987
            marker.pose.pose.position.y = -temp_x_pos + 0.0175
            marker.pose.pose.position.z = -temp_y_pos + 0.10456

            # Applies the transformation.
            marker_transformed = tf2_geometry_msgs.do_transform_pose(
                marker.pose.pose, look_trans)

            # Updates transformstamped message with the marker_transformed.
            t.transform.translation.x = marker_transformed.position.x
            t.transform.translation.y = marker_transformed.position.y
            t.transform.translation.z = marker_transformed.position.z

            # Gets orientation of marker.
            marker_orientation = (marker.pose.pose.orientation.x, marker.pose.pose.orientation.y,
                                  marker.pose.pose.orientation.z, marker.pose.pose.orientation.w)

            # For child frame detected2. Rotate 180 degrees around y-axis.
            # if (t.child_frame_id == '/aruco/detected1'):
            q_rot = quaternion_about_axis(math.pi, (0, -1, 1))
            q = quaternion_multiply(marker_orientation, q_rot)
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]

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
