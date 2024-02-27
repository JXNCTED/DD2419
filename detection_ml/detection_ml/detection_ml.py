#!/usr/bin/env python
import torch
import cv_bridge
from dd2419_detector_baseline.detector import Detector
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class DetectionMLNode(Node):
    def __init__(self):
        super().__init__('arm_controller')
        self.model = Detector().eval().to("cuda")
        self.model.load_state_dict(torch.load(
            "dd2419_detector_baseline/best.pt"))

        self.image_sub = self.create_subscription(
            Image, "/palceholder", self.img_callback)

    def img_callback(self, msg: Image):
        pass


def main():
    rclpy.init()
    node = DetectionMLNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
