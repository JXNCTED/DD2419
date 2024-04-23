import rclpy
import rclpy.duration
from rclpy.node import Node

from detection_interfaces.msg import DetectedObj, Object, StuffList
from detection_interfaces.msg import Stuff as StuffMsg
from collections import deque
from filterpy.kalman import KalmanFilter
import numpy as np
from numpy.linalg import norm

import rclpy.time
from tf2_ros import TransformListener, Buffer
from tf2_geometry_msgs import do_transform_point
from visualization_msgs.msg import Marker, MarkerArray
from detection_interfaces.srv import GetStuff

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
    14: "wc",
}

rv_dict = {v: k for k, v in cls_dict.items()}

cls_dict.update(rv_dict)


class Stuff:
    id = 0

    def __init__(self, position, category, P=100, R=1, Q=0.1):
        self.category = deque(maxlen=10)
        self.gaussian = KalmanFilter(dim_x=2, dim_z=2)
        self.gaussian.x = np.array([position[0], position[1]])
        self.gaussian.F = np.eye(2)
        self.gaussian.H = np.eye(2)
        self.gaussian.P = np.eye(2) * P
        self.gaussian.R = np.eye(2) * R
        self.gaussian.Q = np.eye(2) * Q

        self.id = Stuff.id
        Stuff.id += 1

        self.gaussian.predict()
        self.category.append(category)

    def update(self, category, position):
        self.category.append(category)
        self.gaussian.predict()
        self.gaussian.update(position)

    def residual(self, position):
        return norm(self.gaussian.residual_of(position))

    def getStuff(self):
        # get stuff if the category has more than 5 votes
        max_category = max(self.category, key=self.category.count)
        if self.category.count(max_category) > 5:
            return self.id, max_category, self.gaussian.x
        else:  # return none if the category has less than 5 votes
            return self.id, 0, self.gaussian.x


class CategoryEvaluation(Node):
    def __init__(self):
        super().__init__("category_eval")
        self.get_logger().info("Category evaluation node started")

        self.list_of_stuff: list[Stuff] = []
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.last_stamp = None

        self.create_subscription(
            DetectedObj, "/detection_ml/detected_obj", self.obj_callback, 10
        )

        self.marker_publisher = self.create_publisher(
            MarkerArray, "/category_eval/stuff", 10
        )

        self.prune_timer = self.create_timer(1.0, self.prune_none)

        self.srv = self.create_service(
            GetStuff, "get_stuff", self.get_stuff_callback)

        self.stuff_pub = self.create_publisher(
            StuffList, "/category_eval/stuff_list", 10)

        self.stuff_pub_timer = self.create_timer(1.0, self.publish_stuff_point)

    def publish_stuff_point(self):
        stuff_list = StuffList()
        for stuff in self.list_of_stuff:
            stuff_msg = StuffMsg()
            id, category, position = stuff.getStuff()
            stuff_msg.id = id
            stuff_msg.category = category

            stuff_point = PointStamped()
            stuff_point.header.frame_id = "map"
            stuff_point.header.stamp = self.last_stamp
            stuff_point.point.x = position[0]
            stuff_point.point.y = position[1]
            stuff_point.point.z = 0.0

            stuff_msg.point = stuff_point
            stuff_list.data.append(stuff_msg)
        self.stuff_pub.publish(stuff_list)

    def get_stuff_callback(
        self, request: GetStuff.Request, response: GetStuff.Response
    ):
        if self.list_of_stuff:
            if request.pop:
                self.get_logger().info("pop stuff")
                stuff = self.list_of_stuff.pop()
            else:
                self.get_logger().info("peek stuff")
                stuff = self.list_of_stuff[-1]
            id, category, position = stuff.getStuff()
            response.success = True
            response.stuff_id = id
            response.stuff.position.header.frame_id = "map"
            response.stuff.position.header.stamp = self.last_stamp
            response.stuff.category = category
            response.stuff.confidence = 1.0
            response.stuff.position.point.x = position[0]
            response.stuff.position.point.y = position[1]
            response.stuff.position.point.z = 0.0
            return response
        else:
            response.success = False
            response.stuff = Object()
            return response

    def prune_none(self):
        for stuff in self.list_of_stuff:
            _, category, _ = stuff.getStuff()
            if category == 0:
                self.list_of_stuff.remove(stuff)
                # delete the marker
                markers = MarkerArray()
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = self.last_stamp
                marker.ns = "stuff"
                marker.id = stuff.id
                marker.action = Marker.DELETE
                markers.markers.append(marker)
                text_marker = Marker()
                text_marker.header.frame_id = "map"
                text_marker.header.stamp = self.last_stamp
                text_marker.ns = "stuff"
                text_marker.id = stuff.id + 1000
                text_marker.action = Marker.DELETE
                markers.markers.append(text_marker)
                self.marker_publisher.publish(markers)

    def publish_markers(self):
        markers = MarkerArray()
        for i, stuff in enumerate(self.list_of_stuff):

            id, category, position = stuff.getStuff()
            marker_text = Marker()
            marker_text.header.frame_id = "map"
            marker_text.header.stamp = self.last_stamp
            marker_text.ns = "stuff"
            marker_text.id = id + 1000
            marker_text.type = Marker.TEXT_VIEW_FACING
            marker_text.action = Marker.MODIFY
            marker_text.pose.position.x = position[0]
            marker_text.pose.position.y = position[1]
            marker_text.pose.position.z = 0.1
            marker_text.pose.orientation.x = 0.0
            marker_text.pose.orientation.y = 0.0
            marker_text.pose.orientation.z = 0.0
            marker_text.pose.orientation.w = 1.0
            marker_text.lifetime = rclpy.duration.Duration(seconds=0).to_msg()
            marker_text.scale.x = 0.1
            marker_text.scale.y = 0.1
            marker_text.scale.z = 0.1
            marker_text.color.a = 1.0
            marker_text.color.r = 0.0
            marker_text.color.g = 1.0
            marker_text.color.b = 0.0
            marker_text.text = str(id) + cls_dict[category]
            markers.markers.append(marker_text)

            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.last_stamp
            marker.ns = "stuff"
            marker.id = id
            marker.type = Marker.SPHERE
            marker.action = Marker.MODIFY
            marker.pose.position.x = position[0]
            marker.pose.position.y = position[1]
            marker.pose.position.z = 0.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg()
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            markers.markers.append(marker)

        self.marker_publisher.publish(markers)

    def obj_callback(self, msg: DetectedObj):
        self.last_stamp = msg.header.stamp
        obj: Object
        for obj in msg.obj:
            # skip none and box
            if obj.category == cls_dict["none"] or obj.category == cls_dict["box"]:
                continue
            try:
                # extrapolate into the future??
                t = self.tf_buffer.lookup_transform(
                    "map", msg.header.frame_id, msg.header.stamp, rclpy.duration.Duration(seconds=0.8))
                # t = self.tf_buffer.lookup_transform(
                #     "map", msg.header.frame_id, rclpy.time.Time())

            except Exception as e:
                self.get_logger().error(str(e))
                return

            position = do_transform_point(obj.position, t)

            min_residule = 1000
            min_residule_index = -1
            for i, stuff in enumerate(self.list_of_stuff):
                residule = stuff.residual(
                    np.array([position.point.x, position.point.y]))
                if residule < min_residule:
                    min_residule = residule
                    min_residule_index = i

            if min_residule < 0.15:
                self.list_of_stuff[min_residule_index].update(
                    obj.category, (position.point.x, position.point.y))
            else:
                self.get_logger().info(
                    f"new stuff: {cls_dict[obj.category]} at {position.point.x, position.point.y}"
                )
                self.list_of_stuff.append(
                    Stuff(
                        np.array([position.point.x, position.point.y]), obj.category)
                )

        self.publish_markers()


def main():
    rclpy.init()
    node = CategoryEvaluation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
