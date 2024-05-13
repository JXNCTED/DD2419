#!/usr/bin/env python
import cv_bridge
import torch
from torchvision.transforms import v2
import cv2
from sensor_msgs.msg import Image, CameraInfo
from rclpy.node import Node
import rclpy
from dd2419_detector_baseline.detector import Detector
import time
import numpy as np
from std_msgs.msg import String, Float32MultiArray
import torch_tensorrt
from detection_interfaces.msg import DetectedObj
from detection_interfaces.msg import Object
from geometry_msgs.msg import PointStamped
from realsense2_camera_msgs.msg import RGBD

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


class DetectionMLNode(Node):
    def __init__(self):
        super().__init__('detection_ml')
        self.mode = "front-camera"  # or "arm-camera"
        self.last_time = time.time()

        # This is to export the model to tensorrt

        self.model = Detector().eval().cuda()
        # # i can't figure out how to load the model with launch, so i'm just hardcoding the path
        # self.model.load_state_dict(torch.load(
        #     "/home/group7/best.pt"))

        # self.get_logger().info("Model loaded")

        # inputs = [torch.rand(1, 3, 480, 640).cuda()]

        # self.trt_model = torch_tensorrt.compile(
        #     self.model, ir="ts", inputs=inputs)

        # self.get_logger().info("Model compiled to TensorRT")
        # # save the tensorrt model
        # torch.jit.save(self.trt_model, "/home/group7/trt_model.ts")

        # load the tensorrt model after it's been saved
        self.trt_model = torch.jit.load("/home/group7/trt_model.ts")

        self.val_input_transforms = v2.Compose(
            [
                v2.ToImage(),
                v2.ToDtype(torch.float32, scale=True),
                v2.Normalize(mean=[0.485, 0.456, 0.406],
                             std=[0.229, 0.224, 0.225]),
            ]
        )
        self.get_logger().info("Model loaded")

        # self.K = None

        # self.np_depth = None

        self.K_arm = np.array([[513.34301, 0., 307.89617],
                               [0., 513.84807, 244.62007],
                               [0., 0., 1.]])

        self.coeffs_arm = np.array(
            [-0.474424, 0.207336, -0.002361, 0.000427, 0.000000])

        # self.image_sub = self.create_subscription(
        #     Image, "/camera/color/image_raw", self.img_callback, 10)

        self.image_sub = self.create_subscription(
            Image, "/image_raw", self.arm_img_callback, 5)

        self.arm_timeout_timer = self.create_timer(
            5, self.arm_timeout_callback)

        self.rgbd_sub = self.create_subscription(
            RGBD, "/camera/rgbd", self.realsense_rbgd_callback, 5)

        self.change_mode_sub = self.create_subscription(
            String, "/detection_ml/change_mode", self.change_mode_callback, 5)

        # see the detection_interfaces package for the message definition
        self.detected_obj_pub = self.create_publisher(
            DetectedObj, "/detection_ml/detected_obj", 10)

        self.detection_img_pub = self.create_publisher(
            Image, "/detection_ml/detection_img", 10)

        # publish the bounding box just as an multiarray

        self.bounding_box_pub = self.create_publisher(
            Float32MultiArray, "/detection_ml/bounding_box", 10)

        self.arm_bounding_box_pub = self.create_publisher(
            Float32MultiArray, "/detection_ml/arm_bounding_box", 10)

        # publish the pose of each category. For visualization only, could not handle multiple objects of the same category
        NUM_CLASSES = 15

        self.get_logger().info("Node initialized")

    def change_mode_callback(self, msg: String):
        if msg.data == "front-camera":
            self.mode = "front-camera"
            self.get_logger().info(f"Mode changed to {self.mode}")
        elif msg.data == "arm-camera":
            self.mode = "arm-camera"
            self.get_logger().info(f"Mode changed to {self.mode}")
        else:
            self.get_logger().warn("Invalid mode")
            return

    # project the bounding box to depth and get the position from camera_optical_frame
    def get_position(self, K, depth, bb):
        x, y, w, h = bb[0], bb[1], bb[2], bb[3]
        u = x + w/2
        v = y + h/2
        world_z = depth[int(v), int(u)]
        if world_z == 0 or world_z < 200 or world_z > 1000:
            return np.array([0.0, 0.0, 0.0])
        world_x = (u - K[0, 2]) * world_z / K[0, 0]
        world_y = (v - K[1, 2]) * world_z / K[1, 1]
        return np.array([world_x, world_y, world_z]) / 1000.0

    def realsense_rbgd_callback(self, msg: RGBD):
        if self.mode == "arm-camera":
            return

        K = np.array(msg.rgb_camera_info.k).reshape(3, 3)

        bridge = cv_bridge.CvBridge()
        img = bridge.imgmsg_to_cv2(msg.rgb, "rgb8")
        depth = bridge.imgmsg_to_cv2(msg.depth, "passthrough")

        input_img = torch.stack([self.val_input_transforms(img)]).to(
            "cuda")
        with torch.no_grad():
            out = self.trt_model(input_img)
        DETECT_THRESHOLD = 0.95
        bbs = self.model.out_to_bbs(out, DETECT_THRESHOLD)

        show_img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        IOU_THRESHOLD = 0.5
        # apply non-maximum suppression
        bbs_nms = non_max_suppression(bbs[0], IOU_THRESHOLD)
        length = len(bbs_nms)
        detected_obj = DetectedObj()
        detected_obj.header = msg.rgb.header
        detected_obj.image = msg.rgb

        for bb in bbs_nms:
            x, y, w, h, score, category = int(bb[0]), int(bb[1]), int(
                bb[2]), int(bb[3]), round(bb[4], 2), int(bb[5])

            cata_str = f"{cls_dict[category]}/{category}"
            # draw the bounding box and the category. For visualization only
            cv2.rectangle(show_img, (x, y), (x+w, y+h),
                          color=(0, 255, 0), thickness=2)
            cv2.putText(show_img, cata_str, (x, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            position = self.get_position(K, depth, bb)
            if (position == np.array([0.0, 0.0, 0.0])).all():
                continue
            pose = PointStamped()
            pose.header = msg.rgb.header
            pose.point.x = position[0]
            pose.point.y = position[1]
            pose.point.z = position[2]

            obj = Object()
            obj.category = category
            obj.confidence = score
            obj.position = pose
            obj.bbox = [x, y, w, h]
            detected_obj.obj.append(obj)

        if length > 0:
            self.detected_obj_pub.publish(detected_obj)
            bbs_np = np.array(bbs_nms).flatten()
            self.bounding_box_pub.publish(Float32MultiArray(data=bbs_np))

        fps = round(1/(time.time()-self.last_time), 2)
        self.last_time = time.time()
        status_str = f"FPS: {fps}"
        cv2.putText(show_img, status_str, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        self.detection_img_pub.publish(bridge.cv2_to_imgmsg(show_img))
        # cv2.imshow("detections", show_img)
        # cv2.waitKey(1)

    def arm_timeout_callback(self):
        self.get_logger().warn("Arm camera does not receive image for 5 seconds")

    def arm_img_callback(self, msg: Image):
        self.arm_timeout_timer.reset()
        if self.mode == "front-camera":
            return
        if self.K_arm is None:
            self.get_logger().info("No camera info received yet")
            return

        # convert the image to tensor and run the model
        bridge = cv_bridge.CvBridge()
        img = bridge.imgmsg_to_cv2(msg, "rgb8")
        img = cv2.undistort(img, self.K_arm, self.coeffs_arm)
        img = cv2.blur(img, (5, 5))

        input_img = torch.stack([self.val_input_transforms(img)]).to(
            "cuda")
        with torch.no_grad():
            out = self.trt_model(input_img)
        DETECT_THRESHOLD = 0.95
        bbs = self.model.out_to_bbs(out, DETECT_THRESHOLD)

        show_img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        IOU_THRESHOLD = 0.5
        # apply non-maximum suppression
        bbs_nms = non_max_suppression(bbs[0], IOU_THRESHOLD)
        length = len(bbs_nms)
        detected_obj = DetectedObj()
        detected_obj.header = msg.header

        for bb in bbs_nms:
            x, y, w, h, score, category = int(bb[0]), int(bb[1]), int(
                bb[2]), int(bb[3]), round(bb[4], 2), int(bb[5])

            cata_str = f"{cls_dict[category]}/{category}"
            # draw the bounding box and the category. For visualization only
            # when running, comment out if no valid display is available
            cv2.rectangle(show_img, (x, y), (x+w, y+h),
                          color=(0, 255, 0), thickness=2)
            cv2.putText(show_img, cata_str, (x, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        if length > 0:
            bbs_np = np.array(bbs_nms).flatten()
            self.arm_bounding_box_pub.publish(Float32MultiArray(data=bbs_np))

        fps = round(1/(time.time()-self.last_time), 2)
        self.last_time = time.time()
        status_str = f"FPS: {fps}"
        cv2.putText(show_img, status_str, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        self.detection_img_pub.publish(bridge.cv2_to_imgmsg(show_img))
        # cv2.imshow("detections", show_img)
        # cv2.waitKey(1)


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
