import rclpy
from rclpy.node import Node

from detection_interfaces.msg import DetectedObj
from collections import deque
from dataclasses import dataclass
from filterpy.kalman import KalmanFilter
import numpy as np
from numpy.linalg import norm

import rclpy.time
from tf2_ros import TransformListener, Buffer
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped


cls_dict = {
    0: "none",
    1: "bc",
    2: "binky",
    3: "box",
    4: "bs",
    5: "gc",
    6: "gs",
    7: "hugo",
    8: "kiki",
    9: "muddles",
    10: "oakie",
    11: "rc",
    12: "rs",
    13: "slush",
    14: "wc"
}


@dataclass
class Stuff:
    category: deque = deque(maxlen=5)
    gaussian: KalmanFilter = KalmanFilter(dim_x=2, dim_z=2)

    def __init__(self, position, category, P=100, R=1, Q=0.1):
        self.gaussian.x = np.array([position[0], position[1]])
        self.gaussian.F = np.array([[1, 0], [0, 1]])
        self.gaussian.H = np.array([[1, 0], [0, 1]])
        self.gaussian.P = np.eye(2) * P
        self.gaussian.R = np.eye(2) * R
        self.gaussian.Q = np.eye(2) * Q
        self.gaussian.predict()
        self.category.append(category)

    def update(self, category, position):
        self.category.append(category)
        self.gaussian.predict()
        self.gaussian.update(position)

    def residual(self, position):
        return norm(self.gaussian.residual_of(position))

    def getStuff(self):
        return max(self.category, key=self.category.count), self.gaussian.x


class CategoryEvaluation(Node):
    def __init__(self):
        super().__init__('category_eval')
        self.get_logger().info("Category evaluation node started")

        self.list_of_stuff: list[Stuff] = []
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(
            DetectedObj, '/detection_ml/detected_obj', self.obj_callback, 10
        )

    def obj_callback(self, msg: DetectedObj):
        for obj in msg.obj:
            try:
                t = self.tf_buffer.lookup_transform(
                    "map", "camera_color_optical_frame", rclpy.time.Time())
            except Exception as e:
                self.get_logger().error(str(e))
                return

            position = do_transform_point(obj.position, t)
            for stuff in self.list_of_stuff:
                if stuff.residual(np.array([position.point.x, position.point.y])) < 0.15:
                    stuff.update(
                        obj.category, (position.point.x, position.point.y))
                    break
            else:
                self.get_logger().info(
                    f"new stuff: {cls_dict[obj.category]} at {position.point.x, position.point.y}")
                self.list_of_stuff.append(
                    Stuff((position.point.x, position.point.y), obj.category))

        # self.get_logger().info(f"Number of stuff: {len(self.list_of_stuff)}")


def main():
    rclpy.init()
    node = CategoryEvaluation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
