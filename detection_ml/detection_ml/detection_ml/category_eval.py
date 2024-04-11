import rclpy
from rclpy.node import Node

from detection_interfaces.msg import DetectedObj
from collections import deque

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


class CategoryEvaluation(Node):
    def __init__(self):
        super().__init__('category_eval')
        self.get_logger().info("Category evaluation node started")

        self.last5 = deque(maxlen=5)
        self.create_subscription(
            DetectedObj, '/detection_ml/detected_obj', self.obj_callback, 10
        )

    def obj_callback(self, msg: DetectedObj):
        if len(msg.category) == 0:
            return
        self.last5.append(msg.category[0])  # This line is wrong
        max_count = max(self.last5, key=self.last5.count)

        self.get_logger().info(f"Detected object: {cls_dict[max_count]}")


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
