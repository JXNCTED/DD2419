#!usr/bin/env python3

import rclpy
from rclpy.node import Node

from visualization_msgs.msg import MarkerArray, Marker
from detection_interfaces.msg import DetectedObj

from tf2_ros import TransformListener, Buffer
from robp_interfaces.srv import IsInWorkspace

from tf2_geometry_msgs import do_transform_point


class VisualizeMarkers(Node):
    def __init__(self):
        super().__init__('visualize_markers')
        self.obj_pub = self.create_publisher(
            MarkerArray, '/detection_ml/detected_obj_viz', 10)

        self.obj_sub = self.create_subscription(
            DetectedObj, '/detection_ml/detected_obj', self.detected_obj_callback, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.client = self.create_client(IsInWorkspace, 'is_in_workspace')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.futures = []

    def detected_obj_callback(self, msg: DetectedObj):
        for i in range(len(msg.position)):
            self.is_point_inside_workspace(
                msg.position[i], msg.category[i], msg.header)

    def is_point_inside_workspace(self, position, category, header):
        try:
            t = self.tf_buffer.lookup_transform(
                "map", "camera_color_optical_frame", header.stamp, rclpy.duration.Duration(seconds=0.1))
        except Exception as e:
            self.get_logger().error(str(e))
            return
        request = IsInWorkspace.Request()
        transformed_point = do_transform_point(position, t)
        request.a = transformed_point.point
        future = self.client.call_async(request)
        self.futures.append((position, category, future))
        return future

    def loop(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            marker_array = MarkerArray()
            for position, category, future in self.futures:
                if future.done():
                    response = future.result()
                    marker = Marker()
                    marker.header.frame_id = "camera_color_optical_frame"
                    marker.header.stamp = position.header.stamp
                    marker.id = category
                    marker.type = Marker.CUBE
                    marker.action = Marker.ADD
                    marker.pose.position = position.point
                    marker.pose.orientation.w = 1.0
                    marker.scale.x = 0.1
                    marker.scale.y = 0.1
                    marker.scale.z = 0.1
                    marker.color.r = 1.0 if response.is_in_workspace.data else 0.0
                    marker.color.g = 0.0
                    marker.color.b = 0.0 if response.is_in_workspace.data else 1.0
                    marker.color.a = 1.0
                    marker_array.markers.append(marker)
                    self.get_logger().info(
                        f"Object {category} is in workspace: {response.is_in_workspace}")
                    self.futures.remove((position, category, future))
            self.obj_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = VisualizeMarkers()
    node.loop()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
