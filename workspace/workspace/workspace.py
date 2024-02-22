#!/usr/bin/env python

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point32, PolygonStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Bool 


class Workspace(Node):

    def __init__(self):
        super().__init__('open_loop_controller')
        
        # Define the vertices of the workspace
        self.WORKSPACE = PolygonStamped()
        self.WORKSPACE.polygon.points = [
            Point32(x=0.0, y=0.0, z=0.0),
            Point32(x=5.0, y=0.0, z=0.0),
            Point32(x=1.0, y=1.0, z=0.0),
            Point32(x=0.0, y=1.0, z=0.0)
        ]


        # TODO: Listener: Receives (Nav-)point
        #self.nav_point_subscription = self.create_subscription(
        #    Point32, '/nav_point/point', self.nav_point_callback, 10)
        # TODO: Publisher: Tells if received point is in workspace
        # TODO: Publisher: Broadcast workspace to RVIZ (for visualization)
        
        

        self.workspace_publisher_marker = self.create_publisher(Marker, 'workspace/boundary', 10)
        self.timer_workspace = self.create_timer(1, self.publish_workspace_boundary)

        self.listener = self.create_subscription(
            Point32,
            'workspace/boundary_check/request',
            self.handle_boundary_check,
            10)
        self.check_publisher = self.create_publisher(Bool, 'workspace/boundary_check/response', 10)
        

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
        marker.points.append(Point(x=self.WORKSPACE.polygon.points[0].x, y=self.WORKSPACE.polygon.points[0].y, z=self.WORKSPACE.polygon.points[0].z))

        self.workspace_publisher_marker.publish(marker)


    def is_point_inside_workspace(self, point):

        # Check if a point is inside a polygon using ray-casting algorithm.

        # Args:
        #     point (Point32): The point to check.
        #     polygon (PolygonStamped): The polygon.

        # Returns:
        #     bool: True if the point is inside the polygon, False otherwise.
            
        #num_vertices = len(polygon.polygon.points)
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
            if (p1.y > ray_start.y) != (p2.y > ray_start.y) and (ray_start.x < (p2.x - p1.x) * (ray_start.y - p1.y) / (p2.y - p1.y) + p1.x):
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
