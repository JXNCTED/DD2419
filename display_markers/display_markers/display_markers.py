#!/usr/bin/env python

import rclpy
from rclpy.node import Node

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
from filterpy.kalman import KalmanFilter
from math import cos, sin

import tf2_geometry_msgs

from aruco_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import TransformStamped
from detection_interfaces.msg import BoxList
from detection_interfaces.msg import Box as BoxMsg
from typing import Final

from visualization_msgs.msg import MarkerArray as VisMarkerArray
from visualization_msgs.msg import Marker as VisMarker

from detection_interfaces.srv import GetBox


class Box:
    BOX_WIDTH: Final = 0.16
    BOX_LENGTH: Final = 0.24

    def __init__(self, aruco_id, P=100, R=1, Q=0.1):
        # x, y and yaw of the center of the box
        self.filter = KalmanFilter(dim_x=3, dim_z=3)
        self.filter.x = np.array([0, 0, 0])
        self.filter.F = np.eye(3)
        self.filter.H = np.eye(3)
        self.filter.P = np.eye(3) * P
        self.filter.R = np.eye(3) * R
        self.filter.Q = np.eye(3) * Q

        self.aruco_id = aruco_id  # aruco marker id

    def update(self, position, yaw):
        self.filter.predict()
        self.filter.update(np.array(
            [position[0], position[1], yaw]))

    def get_position(self):
        return np.array([self.filter.x[0], self.filter.x[1], self.filter.x[2]], dtype=np.float64)


class DisplayMarkers(Node):

    def __init__(self):

        super().__init__('display_markers')

        self.ACCEPTABLE_MARKER_IDS: Final = [1, 2, 3]

        self.boxes = {i: Box(i) for i in self.ACCEPTABLE_MARKER_IDS}

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

        self.viz_publisher = self.create_publisher(
            VisMarkerArray, '/box_visualization', 1)
        self.srv = self.create_service(
            GetBox, 'get_box', self.get_box_callback)

        self.box_list_pub = self.create_publisher(
            BoxList, '/box_list', 10)

        self.viz_box_timer = self.create_timer(0.1, self.visualize_box)

    def get_box_callback(self, request: GetBox.Request, response: GetBox.Response):
        self.get_logger().info(f"Received request for box ID {request.box_id}")
        box_id = int(request.box_id.data)
        if box_id not in self.ACCEPTABLE_MARKER_IDS:
            response.success = False
            response.box_pose.header.frame_id = "map"
            response.box_pose.header.stamp = self.get_clock().now().to_msg()
            response.box_pose.pose.position.x = 0
            response.box_pose.pose.position.y = 0
            response.box_pose.pose.position.z = 0
            response.box_pose.pose.orientation.x = 0
            response.box_pose.pose.orientation.y = 0
            response.box_pose.pose.orientation.z = 0
            response.box_pose.pose.orientation.w = 1

            self.get_logger().warn(
                f"Box ID {box_id} not in acceptable box IDs")
            return response

        box = self.boxes[box_id]
        position = box.get_position()
        if position[0] == 0 and position[1] == 0:
            self.get_logger().warn(
                f"Box ID {box_id} not found in the map")
            response.success = False
            response.box_pose.header.frame_id = "map"
            response.box_pose.header.stamp = self.get_clock().now().to_msg()
            response.box_pose.pose.position.x = 0.0
            response.box_pose.pose.position.y = 0.0
            response.box_pose.pose.position.z = 0.0
            response.box_pose.pose.orientation.x = 0.0
            response.box_pose.pose.orientation.y = 0.0
            response.box_pose.pose.orientation.z = 0.0
            response.box_pose.pose.orientation.w = 1.0
            return response
        else:
            response.success = True
            response.box_pose.header.frame_id = "map"
            response.box_pose.header.stamp = self.get_clock().now().to_msg()
            response.box_pose.pose.position.x = position[0]
            response.box_pose.pose.position.y = position[1]
            response.box_pose.pose.position.z = 0.0
            q = quaternion_from_euler(0, 0, position[2])
            response.box_pose.pose.orientation.x = q[0]
            response.box_pose.pose.orientation.y = q[1]
            response.box_pose.pose.orientation.z = q[2]
            response.box_pose.pose.orientation.w = q[3]
            self.get_logger().info(
                f"Box ID {box_id} found at position {position}")
            return response

    def aruco_callback(self, msg: MarkerArray):

        marker: Marker
        for marker in msg.markers:

            if marker.id not in self.ACCEPTABLE_MARKER_IDS:
                self.get_logger().warn(
                    f"Marker ID {marker.id} not in acceptable marker IDs")
                continue

            try:
                look_trans = self.tf_buffer.lookup_transform(
                    "map", "camera_color_optical_frame", rclpy.time.Time())
            except Exception as e:
                self.get_logger().warn(str(e))
                continue

            # Applies the transformation in map frame

            t = TransformStamped()
            t.header.frame_id = "map"
            t.header.stamp = msg.header.stamp
            t.child_frame_id = f"aruco_{marker.id}"

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

            # Updates the box with the new position and orientation
            box_yaw = euler_from_quaternion(
                [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w])[2]
            # THIS IS STRANGE??
            box_center = np.array([
                t.transform.translation.x,
                t.transform.translation.y
            ])

            self.boxes[marker.id].update(box_center, box_yaw)

            # Publish the message.
            try:
                self._tf_broadcaster.sendTransform(t)
            except Exception as e:
                self.get_logger().warn(str(e))
        # display the box in rviz
        # self.visualize_box()

    def visualize_box(self):
        marker_array = VisMarkerArray()
        box_list = BoxList()
        for box in self.boxes.values():
            position = box.get_position()
            if position[0] == 0 and position[1] == 0:
                continue

            marker = VisMarker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "box"
            marker.id = box.aruco_id
            marker.type = VisMarker.CUBE
            marker.action = VisMarker.MODIFY
            marker.pose.position.x = position[0]
            marker.pose.position.y = position[1]
            marker.pose.position.z = 0.0
            box_q = quaternion_from_euler(0, 0, position[2])
            marker.pose.orientation.x = box_q[0]
            marker.pose.orientation.y = box_q[1]
            marker.pose.orientation.z = box_q[2]
            marker.pose.orientation.w = box_q[3]
            marker.scale.x = Box.BOX_LENGTH
            marker.scale.y = Box.BOX_WIDTH
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker_array.markers.append(marker)
            # box_list.boxes.append(marker.pose)
            box_msg = BoxMsg()
            box_msg.aruco_id = box.aruco_id
            box_msg.pose = marker.pose
            box_list.boxes.append(box_msg)
            text_marker = VisMarker()
            text_marker.header.frame_id = "map"
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = "box"
            text_marker.id = box.aruco_id + 100
            text_marker.type = VisMarker.TEXT_VIEW_FACING
            text_marker.action = VisMarker.MODIFY
            text_marker.pose.position.x = position[0]
            text_marker.pose.position.y = position[1]
            text_marker.pose.position.z = 0.2
            text_marker.pose.orientation.w = 1.0
            text_marker.text = f"box_{box.aruco_id}"
            text_marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg()
            text_marker.scale.x = 0.1
            text_marker.scale.y = 0.1
            text_marker.scale.z = 0.1
            text_marker.color.a = 1.0
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            marker_array.markers.append(text_marker)
        self.viz_publisher.publish(marker_array)
        self.box_list_pub.publish(box_list)


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
