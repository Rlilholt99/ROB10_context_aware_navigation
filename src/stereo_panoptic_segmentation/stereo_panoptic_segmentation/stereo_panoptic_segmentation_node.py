#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import message_filters

import os
import warnings
import numpy as np
import cv2
from PIL import Image as PILImage
import torch
from transformers import AutoImageProcessor, Mask2FormerForUniversalSegmentation

import threading
import queue
import time

# Optional: disable HF Hub warning
os.environ["HF_HUB_DISABLE_SYMLINKS_WARNING"] = "1"
warnings.filterwarnings("ignore")

################################################################################
# Environment classification and bounding-box fusion dictionaries
################################################################################
ENV_KEYWORDS = {
    "kitchen":  ["sink", "refrigerator", "cabinet", "oven", "microwave", "glass", "wine glass", "plate", "knife", "fork"],
    "office":   ["desk", "computer", "office chair", "keyboard", "monitor", "Screen", "CRT Screen"],
    "corridor": ["wall", "door", "hallway", "corridor"],
    "lab":      ["pole", "barrel", "truck", "conveyer belt", "washer", "sink", "counter", "desk",
                 "countertop", "stove", "box", "barrel", "basket", "pole", "column", "runway",
                 "escalator", "light", "traffic light", "monitor", "screen", "CRT screen", "wooden pallet",
                 "wooden pallet stack", "wooden Box", "Box", "fire extinguisher"],
    "bedroom":  ["bed", "pillow", "wardrobe", "lamp"],
    "living room": ["sofa", "television", "table", "carpet", "lamp", "ball"]
}

FUSION_CATEGORIES = {
    "kitchen":  ["sink", "refrigerator", "cabinet", "oven", "microwave", "kitchen island"],
    "office":   ["desk", "computer", "office chair", "keyboard", "monitor", "Screen", "CRT Screen"],
    "corridor": ["wall", "door", "hallway", "corridor"],
    "lab":      ["pole", "barrel", "truck", "conveyer belt", "washer", "sink", "counter", "desk",
                 "countertop", "stove", "box", "barrel", "basket", "pole", "column", "runway",
                 "escalator", "light", "traffic light", "monitor", "screen", "CRT screen", "wooden pallet",
                 "wooden pallet stack", "wooden Box", "Box", "fire extinguisher"],
    "bedroom":  ["bed", "pillow", "wardrobe", "lamp"],
    "living room": ["sofa", "television", "table", "carpet", "lamp", "ball"]
}


class StereoSemanticMappingNode(Node):
    """
    A ROS 2 Node that:
      - Subscribes to a single RGB topic (/head_front_camera/rgb/image_raw).
      - Uses a background thread to perform heavy segmentation inference.
      - Subscribes to /head_front_camera/depth/image_raw for depth info.
      - Publishes annotated images to /segmentation/annotated.
    """

    def __init__(self):
        super().__init__('stereo_semantic_mapping_node')

        # Topics and processing interval.
        self.left_topic = '/head_front_camera/rgb/image_raw'
        self.seg_interval = 4

        self.get_logger().info(f"Subscribing camera: {self.left_topic}")
        self.get_logger().info(f"Segmentation every {self.seg_interval} frames")

        # Subscribe to RGB image using message_filters.
        self.sub_left = message_filters.Subscriber(self, Image, self.left_topic)
        self.time_sync = message_filters.ApproximateTimeSynchronizer(
            [self.sub_left], queue_size=10, slop=0.2
        )
        self.time_sync.registerCallback(self.image_callback)

        # Subscribe to depth image.
        self.depth_sub = self.create_subscription(
            Image,
            '/head_front_camera/depth/image_raw',
            self.depth_callback,
            10
        )
        self.depth_image = None

        self.bridge = CvBridge()
        self.frame_count = 0

        # Publisher for annotated images.
        self.ann_pub = self.create_publisher(Image, '/segmentation/annotated', 10)

        # Set up the queue and background thread for processing.
        self.frame_queue = queue.Queue(maxsize=2)
        self.processing_thread = threading.Thread(target=self.process_frames)
        self.processing_thread.daemon = True
        self.processing_thread.start()

        # Set device to GPU if available, otherwise CPU.
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.get_logger().info(f"Using device: {self.device}")

        # Load Mask2Former model.
        self.get_logger().info("Loading Mask2Former model... (may be slow)")
        self.processor = AutoImageProcessor.from_pretrained(
            "facebook/mask2former-swin-large-ade-panoptic", use_fast=False
        )
        self.model = Mask2FormerForUniversalSegmentation.from_pretrained(
            "facebook/mask2former-swin-large-ade-panoptic"
        )
        self.model.to(self.device)
        self.id2label = getattr(self.model.config, "id2label", {})
        self.get_logger().info("Mask2Former loaded.")

        # Stereo matcher and calibration parameters
        self.stereo_matcher = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=16 * 4,
            blockSize=5,
            P1=8 * 3 * 5 ** 2,
            P2=32 * 3 * 5 ** 2,
            disp12MaxDiff=1,
            uniquenessRatio=10,
            speckleWindowSize=100,
            speckleRange=32
        )
        self.focal_length = 800.0
        self.baseline = 0.10

        # Use a lower segmentation resolution to improve speed.
        self.seg_image_size = (1280, 720)

        self.detected_objects = []
        self.environment_type = "unknown"

        self.get_logger().info("Stereo Semantic Mapping Node ready (asynchronous segmentation).")

    def depth_callback(self, msg):
        """
        Store the latest depth image for distance measurements.
        """
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except CvBridgeError as e:
            self.get_logger().error(f"Depth callback bridge error: {e}")
            self.depth_image = None

    def image_callback(self, left_msg):
        """
        Callback for the RGB image.
        Instead of processing here, put the frame in a queue.
        """
        self.frame_count += 1
        if self.frame_count % self.seg_interval != 0:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(left_msg, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge error: {str(e)}")
            return

        # If the queue is full, drop the frame.
        if not self.frame_queue.full():
            self.frame_queue.put(frame)
        else:
            self.get_logger().warn("Frame queue is full, dropping frame.")

    def process_frames(self):
        """
        Worker thread that processes frames from the queue.
        """
        while rclpy.ok():
            try:
                frame = self.frame_queue.get(timeout=1.0)
            except queue.Empty:
                continue

            annotated_img = self.update_semantics(frame, None)
            if annotated_img is not None:
                try:
                    out_msg = self.bridge.cv2_to_imgmsg(annotated_img, encoding="bgr8")
                    self.ann_pub.publish(out_msg)
                    self.get_logger().info("Published annotated image.")
                except CvBridgeError as e:
                    self.get_logger().error(f"CV Bridge error during publish: {str(e)}")
            self.frame_queue.task_done()

    def update_semantics(self, left_cv, depth_map):
        """
        Process the image for object detection and segmentation.
        Uses the stored depth image for distance estimates.
        """
        objects_left, labels_left, annotated_left = self.detect_objects_from_camera(left_cv, depth_map)
        combined_labels = labels_left
        self.environment_type = self.classify_environment(combined_labels)

        all_objects = objects_left
        fused_objects = self.fuse_detections(all_objects)
        self.detected_objects = fused_objects

        combined_annotated = annotated_left

        if combined_annotated is not None:
            cv2.putText(combined_annotated, f"Env: {self.environment_type}",
                        (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)

        self.get_logger().info(f"[Frame {self.frame_count}] Environment: {self.environment_type}")
        if fused_objects:
            for obj in fused_objects:
                dist_str = f"{obj['distance']:.2f}m" if obj.get("distance") else "N/A"
                self.get_logger().info(f"  Object: {obj['label']}, distance: {dist_str}, bbox: {obj['bbox']}")
        else:
            self.get_logger().info("  No objects detected.")

        return combined_annotated

    def detect_objects_from_camera(self, cv_image, dummy_depth_map):
        """
        Runs object detection and segmentation on the provided image.
        Annotates bounding boxes and calculates object distance using the stored depth image.
        """
        # Convert BGR -> RGB, then to PIL image and resize.
        pil_img = PILImage.fromarray(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))
        pil_img = pil_img.resize(self.seg_image_size, PILImage.BILINEAR)

        inputs = self.processor(images=pil_img, return_tensors="pt")
        # Move inputs to the appropriate device.
        inputs = {k: v.to(self.device) for k, v in inputs.items()}
        with torch.no_grad():
            outputs = self.model(**inputs)

        panoptic_res = self.processor.post_process_panoptic_segmentation(
            outputs, target_sizes=[pil_img.size[::-1]]
        )[0]
        segmentation = panoptic_res["segmentation"].cpu().numpy().astype(np.int32)
        segments_info = panoptic_res["segments_info"]

        annotated_img = np.array(pil_img, dtype=np.uint8)
        annotated_img = cv2.cvtColor(annotated_img, cv2.COLOR_RGB2BGR)

        detections = []
        recognized_labels = []

        for seg_info in segments_info:
            seg_id = seg_info["id"]
            label_id = seg_info["label_id"]

            if str(label_id) in self.id2label:
                label_name = self.id2label[str(label_id)]
            elif label_id in self.id2label:
                label_name = self.id2label[label_id]
            else:
                label_name = f"Label_{label_id}"

            mask_y, mask_x = np.where(segmentation == seg_id)
            if mask_y.size == 0 or mask_x.size == 0:
                continue
            y_min, y_max = mask_y.min(), mask_y.max()
            x_min, x_max = mask_x.min(), mask_x.max()

            recognized_labels.append(label_name)

            # Use the stored depth image for distance estimates.
            if self.depth_image is not None:
                dh, dw = self.depth_image.shape[:2]
                y_max_clamped = min(y_max, dh - 1)
                x_max_clamped = min(x_max, dw - 1)
                depth_region = self.depth_image[y_min:y_max_clamped+1, x_min:x_max_clamped+1]
                valid_depths = depth_region[depth_region > 0]
                if valid_depths.size > 0:
                    median_depth = float(np.median(valid_depths))
                else:
                    median_depth = None
            else:
                median_depth = None

            detections.append({
                "label": label_name,
                "bbox": (x_min, y_min, x_max, y_max),
                "distance": median_depth
            })

            dist_str = f"{median_depth:.2f}m" if median_depth else "?"
            annotation_txt = f"{label_name} {dist_str}"
            # Do not draw bounding box for floor or wall, but still include them in detections.
            if label_name.lower() not in ["floor", "wall"]:
                cv2.rectangle(annotated_img, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
                cv2.putText(annotated_img, annotation_txt,
                            (x_min, max(y_min - 10, 0)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        return detections, recognized_labels, annotated_img

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
