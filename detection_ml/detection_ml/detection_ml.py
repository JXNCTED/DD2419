#!/usr/bin/env python
import torch
import cv_bridge
from dd2419_detector_baseline.detector import Detector
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2


class DetectionMLNode(Node):
    def __init__(self):
        super().__init__('arm_controller')
        self.model = Detector().eval().to("cuda")
        self.model.load_state_dict(torch.load(
            "dd2419_detector_baseline/best.pt"))

        self.image_sub = self.create_subscription(
            Image, "/palceholder", self.img_callback)

    def img_callback(self, msg: Image):
        bridge = cv_bridge.CvBridge()
        img = bridge.imgmsg_to_cv2(msg, "bgr8")
        img = torch.from_numpy(img).permute(2, 0, 1).float().to("cuda") / 255.0
        img = img.unsqueeze(0)
        with torch.no_grad():
            detections = self.model(img, augment=False)[0]
        bbs = self.model.out_to_bbs(detections, 0.5)

        show_img = img[0].permute(1, 2, 0).cpu().numpy()
        show_img = cv2.cvtColor(show_img, cv2.COLOR_BGR2RGB)
        for bb in bbs[0]:
            x, y, w, h = bb["x"], bb["y"], bb["width"], bb["height"]
            cv2.rectangle(show_img, (x, y), (x+w, y+h), (0, 255, 0), 2)
        cv2.imshow("detections", show_img)
        cv2.waitKey(1)
        
        


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
