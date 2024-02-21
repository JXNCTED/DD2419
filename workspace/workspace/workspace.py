#!/usr/bin/env python

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point


class Workspace(Node):

    def __init__(self):
        super().__init__('open_loop_controller')
        
        # TODO: Listener: Receives (Nav-)point
        self.nav_point_subscription = self.create_subscription(
            Point, '/nav_point/point', self.nav_point_callback, 10)
        # TODO: Publisher: Tells if received point is in workspace
        # TODO: Publisher: Broadcast workspace to RVIZ (for visualization) 
        
    # TODO: Function?: Hard-Code workspace
    def workspace_boundaries(self):
        #Line 1
        line1 
        pass
    
    # TODO: Function: Check if point is in workspace
    


    def nav_point_callback(msg):
        # Call isInWorkspace(msg)
        # publish answer
        pass


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