#!/usr/bin/env python3
import os

os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '{message}'
os.environ['TRANSFORMERS_VERBOSITY'] = 'error'  # Optionally set environment variable

from transformers import logging as hf_logging

hf_logging.set_verbosity_error()  # This will suppress INFO messages from transformers

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import message_filters
import warnings
import numpy as np
import cv2
from PIL import Image as PILImage
import torch
from transformers import AutoImageProcessor, Mask2FormerForUniversalSegmentation
import threading
import queue
import time
from collections import deque

# Import custom message and necessary geometry messages
from context_aware_nav_interfaces.msg import ObjectLocalPose
from geometry_msgs.msg import Pose, Point, Quaternion

# Optional: disable HF Hub warning
os.environ["HF_HUB_DISABLE_SYMLINKS_WARNING"] = "1"
warnings.filterwarnings("ignore")

################################################################################
# Environment classification and bounding-box fusion dictionaries
################################################################################
ENV_KEYWORDS = {
    "kitchen": ["sink", "refrigerator", "cabinet", "oven", "microwave", "glass", "wine glass", "plate", "knife",
                "fork"],
    "office": ["desk", "computer", "office chair", "keyboard", "monitor", "Screen", "CRT Screen"],
    "corridor": ["wall", "door", "hallway", "corridor"],
    "lab": ["pole", "barrel", "truck", "conveyer belt", "washer", "sink", "counter", "desk",
            "countertop", "stove", "box", "barrel", "basket", "pole", "column", "runway",
            "escalator", "light", "traffic light", "monitor", "screen", "CRT screen", "wooden pallet",
            "wooden pallet stack", "wooden Box", "Box", "fire extinguisher"],
    "bedroom": ["bed", "pillow", "wardrobe", "lamp"],
    "living room": ["sofa", "television", "table", "carpet", "lamp", "ball"]
}

FUSION_CATEGORIES = {
    "kitchen": ["sink", "refrigerator", "cabinet", "oven", "microwave", "kitchen island"],
    "office": ["desk", "computer", "office chair", "keyboard", "monitor", "Screen", "CRT Screen"],
    "corridor": ["wall", "door", "hallway", "corridor"],
    "lab": ["pole", "barrel", "truck", "conveyer belt", "washer", "sink", "counter", "desk",
            "countertop", "stove", "box", "barrel", "basket", "pole", "column", "runway",
            "escalator", "light", "traffic light", "monitor", "screen", "CRT screen", "wooden pallet",
            "wooden pallet stack", "wooden Box", "Box", "fire extinguisher"],
    "bedroom": ["bed", "pillow", "wardrobe", "lamp"],
    "living room": ["sofa", "television", "table", "carpet", "lamp", "ball"]
}


class StereoSemanticMappingNode(Node):
    def __init__(self):
        super().__init__('stereo_semantic_mapping_node')
        # Topics and processing interval
        self.left_topic = '/head_front_camera/rgb/image_raw'
        self.seg_interval = 4

        # Subscribe to RGB image using message_filters
        self.sub_left = message_filters.Subscriber(self, Image, self.left_topic)
        self.time_sync = message_filters.ApproximateTimeSynchronizer(
            [self.sub_left], queue_size=10, slop=0.2)
        self.time_sync.registerCallback(self.image_callback)

        # Subscribe to depth image
        self.depth_sub = self.create_subscription(
            Image,
            '/head_front_camera/depth/image_raw',  # Depth camera range: 0.3m to 8m
            self.depth_callback,
            10)
        self.depth_image = None

        # Subscribe to camera info to get intrinsics
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/head_front_camera/rgb/camera_info',
            self.camera_info_callback,
            10)
        # Initialize camera intrinsics variables
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        # Other initialization (bridge, model loading, etc.)
        self.bridge = CvBridge()
        self.frame_count = 0
        # To only buffer the most recent message (and thus publish every new image), set the QoS depth to 1:
        self.ann_pub = self.create_publisher(Image, '/segmentation/annotated', QoSProfile(depth=1))
        # Publisher for custom ObjectLocalPose message
        self.pose_pub = self.create_publisher(ObjectLocalPose, '/object_local_pose', QoSProfile(depth=1))


        # This part looks in if you have Cuda setup it will use it if not CPU for you model segmantion
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.get_logger().info(f"Using device: {self.device}")

        self.get_logger().info("Loading Mask2Former model... (may be slow)")
        self.processor = AutoImageProcessor.from_pretrained(
            "facebook/mask2former-swin-large-ade-panoptic", use_fast=False)
        self.model = Mask2FormerForUniversalSegmentation.from_pretrained(
            "facebook/mask2former-swin-large-ade-panoptic")
        self.model.to(self.device)


        # Set label_ids_to_fuse to an empty list or your desired configuration
        self.model.config.label_ids_to_fuse = []
        self.id2label = getattr(self.model.config, "id2label", {})
        self.get_logger().info("Mask2Former loaded.")

        # Use a lower segmentation resolution to improve speed.
        self.seg_image_size = (640, 480)
        self.detected_objects = []
        self.environment_type = "unknown"


        #Store Last Published Poses:
        self.last_published_poses = {}  # Dictionary: key = object label, value = (X, Y, Z)
        self.publish_threshold = 0.05  # Threshold (in meters) to decide if a change is significant


        # ****************************************
        # *** Confidence Threshold Setup ***
        self.confidence_threshold = 0.5
        # ****************************************


    #This callback receives the camera info (intrinsics) and sets the internal variables self.fx, self.fy, self.cx, and self.cy.
    # These are essential for converting 2D pixel coordinates and depth to 3D positions
    def camera_info_callback(self, msg):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    ############################################################################################
    #       depth callback
    ############################################################################################
    #This callback converts incoming depth image messages to a CV2 image using CvBridge and stores it in self.depth_image. If conversion fails, an error is logged.
    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except CvBridgeError as e:
            self.get_logger().error(f"Depth callback bridge error: {e}")
            self.depth_image = None

    ############################################################################################
    #       end depth callback
    ############################################################################################
    ############################################################################################
    #       image callback
    ############################################################################################
    #This callback is triggered by synchronized RGB images.
    # It converts the ROS image to an OpenCV (cv2) format, then immediately calls update_semantics(frame, None) to process the image:
    def image_callback(self, left_msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(left_msg, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge error: {str(e)}")
            return

        # Directly process the frame.
        annotated_img = self.update_semantics(frame, None)
        if annotated_img is not None:
            try:
                out_msg = self.bridge.cv2_to_imgmsg(annotated_img, encoding="bgr8")
                self.ann_pub.publish(out_msg)
            except CvBridgeError as e:
                self.get_logger().error(f"CV Bridge error during publish: {str(e)}")

    ############################################################################################
    #       end image callback
    ############################################################################################

    ############################################################################################
    #       Update semantics
    ############################################################################################
    def update_semantics(self, left_cv, depth_map):
        # Run segmentation and obtain detections, labels, and annotated image.
        objects_left, labels_left, annotated_left = self.detect_objects_from_camera(left_cv, depth_map)
        combined_labels = labels_left
        self.environment_type = self.classify_environment(combined_labels)
        fused_objects = self.fuse_detections(objects_left)
        self.detected_objects = fused_objects

        # 2D Position Estimation: Compute object's 3D position from bounding box and depth.
        if self.fx is None or self.fy is None or self.cx is None or self.cy is None:
            self.get_logger().warn("Camera intrinsics not received yet!")
        else:
            for obj in fused_objects:
                x_min, y_min, x_max, y_max = obj["bbox"]
                u = (x_min + x_max) / 2.0
                v = (y_min + y_max) / 2.0
                d = obj["distance"]
                if d is not None:
                    X = (u - self.cx) * d / self.fx
                    Y = (v - self.cy) * d / self.fy
                    Z = d
                else:
                    X, Y, Z = None, None, None
                obj["position"] = (X, Y, Z)

        # Annotate image with environment information.
        if annotated_left is not None:
            cv2.putText(annotated_left, f"Env: {self.environment_type}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)

        # Log the environment and each object's computed position.
        self.get_logger().info(f"Environment: {self.environment_type}")
        for obj in fused_objects:
            pos = obj.get("position", "Not computed")
            if obj["distance"] is not None:
                self.get_logger().info(f"Object {obj['label']} Position: {pos}, Distance: {obj['distance']:.2f}m")
            else:
                self.get_logger().info(f"Object {obj['label']} Position: Not computed")

        # Build the ObjectLocalPose message.
        local_pose_msg = ObjectLocalPose()
        local_pose_msg.object_labels = []
        local_pose_msg.object_pose = []

        # For each detection, fill in the label and pose.
        for obj in fused_objects:
            label = obj["label"]
            if obj.get("position") is not None and None not in obj["position"]:
                current_pose = obj["position"]
            else:
                current_pose = (0.0, 0.0, 0.0)
            local_pose_msg.object_labels.append(label)
            pose = Pose()
            # Use keyword arguments to create the Point message.
            pose.position = Point(x=current_pose[0], y=current_pose[1], z=current_pose[2])
            pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            local_pose_msg.object_pose.append(pose)

        # Publish the new message every time update_semantics is executed.
        self.pose_pub.publish(local_pose_msg)

        # Resize the annotated image for viewing.
        view_resolution = (1280, 720)
        annotated_view = cv2.resize(annotated_left, view_resolution, interpolation=cv2.INTER_LINEAR)
        return annotated_view


    ############################################################################################
    #      End Update semantics
    ############################################################################################
    ############################################################################################
    #       Detect object from camera
    ############################################################################################
    def detect_objects_from_camera(self, cv_image, dummy_depth_map):
        # Convert the OpenCV image to a PIL image and resize it to the segmentation resolution.
        pil_img = PILImage.fromarray(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))
        pil_img = pil_img.resize(self.seg_image_size, PILImage.BILINEAR)

        # Run the image through Mask2Former.
        inputs = self.processor(images=pil_img, return_tensors="pt")
        inputs = {k: v.to(self.device) for k, v in inputs.items()}
        with torch.no_grad():
            outputs = self.model(**inputs)
        panoptic_res = self.processor.post_process_panoptic_segmentation(
            outputs, target_sizes=[pil_img.size[::-1]]
        )[0]
        segmentation = panoptic_res["segmentation"].cpu().numpy().astype(np.int32)
        segments_info = panoptic_res["segments_info"]

        # Convert back to an OpenCV image for annotation.
        annotated_img = np.array(pil_img, dtype=np.uint8)
        annotated_img = cv2.cvtColor(annotated_img, cv2.COLOR_RGB2BGR)

        detections = []
        recognized_labels = []

        for seg_info in segments_info:
            seg_id = seg_info["id"]
            label_id = seg_info["label_id"]

            # Determine the label name using the model's id2label mapping.
            if str(label_id) in self.id2label:
                label_name = self.id2label[str(label_id)]
            elif label_id in self.id2label:
                label_name = self.id2label[label_id]
            else:
                continue

            # Use confidence score filtering.
            confidence = seg_info.get("score", 1.0)
            if confidence < self.confidence_threshold:
                continue
            label_name_with_conf = f"{label_name} ({confidence * 100:.1f}%)"

            # Retrieve the segmentation mask indices at the segmentation resolution.
            mask_indices = np.where(segmentation == seg_id)
            if mask_indices[0].size == 0 or mask_indices[1].size == 0:
                continue
            y_min, y_max = mask_indices[0].min(), mask_indices[0].max()
            x_min, x_max = mask_indices[1].min(), mask_indices[1].max()

            # Add the label (with confidence) to the recognized list.
            recognized_labels.append(label_name_with_conf)
            ########################
            #    start Depth
            ########################
            # Depth extraction using the segmentation mask.
            if self.depth_image is not None:
                depth_h, depth_w = self.depth_image.shape[:2]
                # Create a binary mask at the segmentation resolution.
                seg_mask = (segmentation == seg_id).astype(np.uint8)
                # Resize the segmentation mask to the depth image resolution (640x480) using nearest neighbor.
                mask_resized = cv2.resize(seg_mask, (depth_w, depth_h), interpolation=cv2.INTER_NEAREST)
                mask_resized = mask_resized.astype(bool)

                # Extract depth values from the depth image using the resized mask.
                valid_depths = self.depth_image[mask_resized]
                valid_depths = valid_depths[(valid_depths > 0) & np.isfinite(valid_depths)]
                if valid_depths.size > 0:
                    median_depth = float(np.median(valid_depths))
                    if (not np.isfinite(median_depth)) or (median_depth < 0.3) or (median_depth > 8.0):
                        median_depth = None
                else:
                    median_depth = None
            else:
                median_depth = None
            ########################
            #   end Depth
            #########################
            # Build annotation strings.
            dist_str = f"{median_depth:.2f}m" if median_depth is not None else "out of range"
            annotation_txt = f"{label_name_with_conf} {dist_str}"

            # Ignore certain classes (e.g., floor, wall, etc.).
            if label_name.lower() in ["floor", "wall", "ceiling", "background", "void", "sidewalk",
                                      "hallway", "earth", "building", "sky"]:
                continue

            # Draw the bounding box and annotation for visualization.
            cv2.rectangle(annotated_img, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
            cv2.putText(annotated_img, annotation_txt,
                        (x_min, max(y_min - 10, 0)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            # Append the detection information.
            detections.append({
                "label": label_name,
                "bbox": (x_min, y_min, x_max, y_max),
                "distance": median_depth
            })
        return detections, recognized_labels, annotated_img

    ############################################################################################
    #      End Detect object from camera
    ############################################################################################
    ############################################################################################
    #      duse detection
    ############################################################################################
    #f several detections belong to a fusion category, this part of the code creates one bounding box that covers all those detections and averages their distance values.
    # This helps to avoid redundant or overlapping detections for the same object type.
    def fuse_detections(self, detections):
        fused = []
        groups = {}
        others = []
        for det in detections:
            label_lower = det["label"].lower().strip()
            found = False
            for cat, words in FUSION_CATEGORIES.items():
                if any(w in label_lower for w in words):
                    groups.setdefault(cat, []).append(det)
                    found = True
                    break
            if not found:
                others.append(det)
        for cat, group in groups.items():
            if len(group) == 1:
                fused.append(group[0])
            else:
                x_min = min(d["bbox"][0] for d in group)
                y_min = min(d["bbox"][1] for d in group)
                x_max = max(d["bbox"][2] for d in group)
                y_max = max(d["bbox"][3] for d in group)
                distances = [d["distance"] for d in group if d["distance"] is not None]
                avg_dist = sum(distances) / len(distances) if distances else None
                fused.append({
                    "label": cat,
                    "bbox": (x_min, y_min, x_max, y_max),
                    "distance": avg_dist
                })
        return fused + others
    ############################################################################################
    #      End fuse detections
    ############################################################################################
    def classify_environment(self, labels_found):
        env_counts = {env: 0 for env in ENV_KEYWORDS}
        for label in labels_found:
            for env, keywords in ENV_KEYWORDS.items():
                for kw in keywords:
                    if kw.lower() in label.lower():
                        env_counts[env] += 1
        best_env = max(env_counts, key=env_counts.get)
        return best_env if env_counts[best_env] > 0 else "unknown"

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = StereoSemanticMappingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt -> shutting down.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

    # TODO:
    # make sure that when you do Temporal Smoothing Setup for classfiation only and not moving average over fram.
    # update every time their is a new image then queue max 1.
    # every time this happing segmentation publish
    # what i want now is to make sure that every time i do labing if i have 2 the same type of label they have different id.
    # smantic mapping 
