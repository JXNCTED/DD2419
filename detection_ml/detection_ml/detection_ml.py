#!/usr/bin/env python
import cv_bridge
import torch
from torchvision.transforms import v2
import cv2
from sensor_msgs.msg import Image
from rclpy.node import Node
import rclpy
from dd2419_detector_baseline.detector import Detector, BoundingBox
import time
import numpy as np


class DetectionMLNode(Node):
    def __init__(self):
        super().__init__('detection_ml')
        self.model = Detector().eval().to("cuda")
        # i can't figure out how to load the model with launch, so i'm just hardcoding the path
        self.model.load_state_dict(torch.load(
            "/home/group7/best.pt"))

        self.image_sub = self.create_subscription(
            Image, "/camera/color/image_raw", self.img_callback, 10)

    def img_callback(self, msg: Image):
        start = time.time()
        val_input_transforms = v2.Compose(
            [
                v2.ToImage(),
                v2.ToDtype(torch.float32, scale=True),
                v2.Normalize(mean=[0.485, 0.456, 0.406],
                             std=[0.229, 0.224, 0.225]),
            ]
        )
        bridge = cv_bridge.CvBridge()
        img = bridge.imgmsg_to_cv2(msg, "rgb8")
        input_img = torch.stack([val_input_transforms(img)]).to("cuda")
        with torch.no_grad():
            out = self.model(input_img).cpu()
        DETECT_THRESHOLD = 0.5
        bbs = self.model.out_to_bbs(out, DETECT_THRESHOLD)

        show_img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        IOU_THRESHOLD = 0.5
        bbs_nms = non_max_suppression(bbs[0], IOU_THRESHOLD)
        for bb in bbs_nms:
            print(bb.shape)
            x, y, w, h, score, category = int(bb[0]), int(bb[1]), int(
                bb[2]), int(bb[3]), round(bb[4], 2), int(bb[5])

            cata_str = f"{category} scr:{score}"
            cv2.rectangle(show_img, (x, y), (x+w, y+h),
                          color=(0, 255, 0), thickness=2)
            cv2.putText(show_img, cata_str, (x, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        fps = round(1/(time.time()-start), 2)
        cv2.putText(show_img, f"FPS: {fps}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.imshow("detections", show_img)
        cv2.waitKey(1)


def non_max_suppression(boxes, threshold):
    """
    Perform non-maximum suppression on the bounding boxes.
    :param boxes: A list of bounding box dictionaries.
    :param threshold: The threshold for the overlap ratio.
    :return: np array of the picked boxes, in the format of [x, y, w, h, score, category]
    """
    if len(boxes) == 0:
        return []

    # Convert the list of bounding box dictionaries to a NumPy array
    boxes_array = np.array(
        [(box['x'], box['y'], box['width'], box['height'], box['score'], box['category']) for box in boxes])

    # Sort the boxes by their scores in descending order
    sorted_indices = np.argsort(boxes_array[:, 4])[::-1]
    boxes_array = boxes_array[sorted_indices]

    picked_boxes = []
    while len(boxes_array) > 0:
        # Pick the box with the highest score
        top_box = boxes_array[0]
        picked_boxes.append(top_box)

        # Calculate the coordinates of the intersection rectangle
        x1 = np.maximum(top_box[0], boxes_array[1:, 0])
        y1 = np.maximum(top_box[1], boxes_array[1:, 1])
        x2 = np.minimum(top_box[0] + top_box[2],
                        boxes_array[1:, 0] + boxes_array[1:, 2])
        y2 = np.minimum(top_box[1] + top_box[3],
                        boxes_array[1:, 1] + boxes_array[1:, 3])

        # Calculate the area of overlap
        intersection_area = np.maximum(0, x2 - x1) * np.maximum(0, y2 - y1)
        box_area = (boxes_array[1:, 2] * boxes_array[1:, 3])

        # Calculate the overlap ratio
        overlap_ratio = intersection_area / box_area

        # Filter out boxes with high overlap
        filtered_indices = np.where(overlap_ratio <= threshold)[0]
        boxes_array = boxes_array[1 + filtered_indices]

    return picked_boxes


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
