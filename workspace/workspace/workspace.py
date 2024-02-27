#!/usr/bin/env python

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point32, PolygonStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Bool

import csv


class Workspace(Node):

    def __init__(self):
        super().__init__('open_loop_controller')

        # Define the vertices of the workspace
        self.WORKSPACE = PolygonStamped()
        # self.WORKSPACE.polygon.points = [
        #     Point32(x=0.0, y=0.0, z=0.0),
        #     Point32(x=5.0, y=0.0, z=0.0),
        #     Point32(x=1.0, y=1.0, z=0.0),
        #     Point32(x=0.0, y=1.0, z=0.0)
        # ]
        self.read_workspace_points()

        self.workspace_publisher_marker = self.create_publisher(Marker, 'workspace/boundary', 10)
        self.timer_workspace = self.create_timer(1, self.publish_workspace_boundary)

        self.listener = self.create_subscription(
            Point32,
            'workspace/boundary_check/request',
            self.handle_boundary_check,
            10)
        self.check_publisher = self.create_publisher(Bool, 'workspace/boundary_check/response', 10)

    def read_workspace_points(self):
        # Specify the file path
        file_path = "workspace_points.tsv"

        # Create variable to store points
        points_list = []

        # Open the TSV file for reading
        with open(file_path, mode='r', newline='') as file:
            # Create a CSV reader object with tab delimiter
            tsv_reader = csv.reader(file, delimiter='\t')

            # Skip the header row where only x and y is written
            next(tsv_reader)

            # Iterate over each row in the TSV file
            for row in tsv_reader:
                # Write the workspace_points into the workspace
                x, y, z = float(row[0]), float(row[1]), float(0.0)
                points_list.append(Point32(x=x, y=y, z=z))

        # Add the points to the workspace
        self.WORKSPACE.polygon.points = points_list

    def publish_workspace_boundary(self):

        marker = Marker()
        marker.header.frame_id = "map"  # Set the frame in which the polygon will be displayed
        marker.type = Marker.LINE_STRIP  # Set the marker type to LINE_STRIP for polygon
        marker.action = Marker.ADD
        marker.scale.x = 0.1  # Set the thickness of the lines
        marker.color.r = 0.5
        marker.color.g = 0.0
        marker.color.b = 0.5
        marker.color.a = 1.0  # Set the alpha (transparency)
        marker.pose.orientation.w = 1.0  # Set the orientation (quaternion)

        marker.points = [Point(x=point.x, y=point.y, z=point.z) for point in self.WORKSPACE.polygon.points]
        marker.points.append(Point(x=self.WORKSPACE.polygon.points[0].x, y=self.WORKSPACE.polygon.points[0].y,
                                   z=self.WORKSPACE.polygon.points[0].z))

        self.workspace_publisher_marker.publish(marker)

    def is_point_inside_workspace(self, point):

        # Check if a point is inside a polygon using ray-casting algorithm.

        # Args:
        #     point (Point32): The point to check.
        #     polygon (PolygonStamped): The polygon.

        # Returns:
        #     bool: True if the point is inside the polygon, False otherwise.

        # num_vertices = len(polygon.polygon.points)
        num_vertices = len(self.WORKSPACE.polygon.points)
        inside = False

        # Create a ray starting from the point and extending to the right
        ray_start = point
        ray_end = Point32(x=1e6, y=point.y, z=0.0)  # Extend ray to a very large x value

        # Count intersections of the ray with the edges of the polygon
        for i in range(num_vertices):
            p1 = self.WORKSPACE.polygon.points[i]
            p2 = self.WORKSPACE.polygon.points[(i + 1) % num_vertices]

            # Check if the ray intersects the edge
            if (p1.y > ray_start.y) != (p2.y > ray_start.y) and (
                    ray_start.x < (p2.x - p1.x) * (ray_start.y - p1.y) / (p2.y - p1.y) + p1.x):
                inside = not inside
        r = Bool()
        r.data = inside
        return r

    def handle_boundary_check(self, msg):
        inside = self.is_point_inside_workspace(msg)
        self.check_publisher.publish(inside)


def main():
    rclpy.init()
    node = Workspace()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()

### Not used anymore:

# self.workspace_publisher = self.create_publisher(PolygonStamped, 'polygon', 10)
# self.timer_workspace = self.create_timer(1, self.publish_workspace)

# def publish_workspace(self):
#     polygon = PolygonStamped()
#     #polygon.header.stamp = self.Time.now()
#     polygon.header.frame_id = "map"
#     polygon.polygon.points = self.WORKSPACE.polygon.points
#     self.workspace_publisher.publish(polygon)
