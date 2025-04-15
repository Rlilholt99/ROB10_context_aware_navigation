#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import torch
from PIL import Image as PILImage
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry
import message_filters
import math

from transformers import AutoImageProcessor, Mask2FormerForUniversalSegmentation

class PanopticSegmentationNode(Node):
    def __init__(self):
        super().__init__('panoptic_segmentation_node')

        # --- Subscribers for stereo cameras (synchronized) ---
        self.sub_left = message_filters.Subscriber(self, Image, '/stereo/left/image_raw')
        self.sub_right = message_filters.Subscriber(self, Image, '/stereo/right/image_raw')
        self.ts = message_filters.ApproximateTimeSynchronizer([self.sub_left, self.sub_right],
                                                              queue_size=10, slop=0.1)
        self.ts.registerCallback(self.image_callback)

        # --- Additional Subscribers for LiDAR and Odometry ---
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # --- Publisher for the annotated/fused image ---
        self.publisher = self.create_publisher(Image, '/segmentation/annotated', 10)

        # --- CV Bridge ---
        self.bridge = CvBridge()

        # --- Load Segmentation Model and Processor ---
        # You can choose the model that best fits your application.
        self.processor = AutoImageProcessor.from_pretrained(
            "facebook/mask2former-swin-base-coco-panoptic", use_fast=False
        )
        self.model = Mask2FormerForUniversalSegmentation.from_pretrained(
            "facebook/mask2former-swin-large-coco-panoptic"
        )
        self.model.to("cpu")  # Change to "cuda" if GPU is available

        # --- Parameters ---
        self.segmentation_interval = 10  # Only process every Nth frame for performance.
        self.frame_count = 0
        self.camera_fov = 1.0472  # Field-of-view (in radians) used for mapping image pixel to LiDAR angle.
        self.image_resolution = (1280, 720)  # (width, height) used in segmentation

        # --- Environment Keywords for Classification ---
        self.ENV_KEYWORDS = {
            "kitchen":  ["sink", "refrigerator", "cabinet", "oven", "microwave", "glass", "wine glass", "plate", "knife", "fork"],
            "office":   ["desk", "computer", "office chair", "keyboard", "monitor", "Screen", "CRT Screen"],
            "corridor": ["wall", "door", "hallway", "corridor"],
            "lab":      ["pole", "barrel", "truck", "conveyer belt", "washer", "sink", "counter", "desk", "countertop",
                         "stove", "box", "basket", "column", "runway", "escalator", "light", "traffic light", "monitor",
                         "screen", "CRT screen", "wooden pallet", "wooden pallet stack", "wooden Box", "Box", "fire extinguisher"],
            "bedroom":  ["bed", "pillow", "wardrobe", "lamp"],
            "living room": ["sofa", "television", "table", "carpet", "lamp", "ball"],
        }

        # --- Placeholders for latest LiDAR scan and odometry ---
        self.latest_scan = None
        self.latest_odom = None

        self.get_logger().info("Panoptic Segmentation Node initialized.")

    def scan_callback(self, scan_msg):
        # Store latest LiDAR scan for use in distance estimation.
        self.latest_scan = scan_msg

    def odom_callback(self, odom_msg):
        # Store latest odometry message.
        self.latest_odom = odom_msg

    def estimate_distance(self, cx, image_width):
        """
        Estimate the distance to an object given its center x-coordinate in the image.
        This function maps the x coordinate to an angle (assuming the camera's FOV)
        and then looks up the corresponding LiDAR range if available.
        """
        if self.latest_scan is None:
            return None

        half_width = image_width / 2.0
        # Calculate offset angle based on where the object appears in the image.
        angle_offset = (cx - half_width) / half_width * (self.camera_fov / 2.0)
        # Use LiDAR parameters to find the index:
        scan = self.latest_scan
        target_angle = angle_offset  # Assuming camera's forward direction corresponds to 0 rad
        # Ensure target_angle is within the LiDAR scan range.
        if target_angle < scan.angle_min or target_angle > scan.angle_max:
            return None
        index = int((target_angle - scan.angle_min) / scan.angle_increment)
        if index < 0 or index >= len(scan.ranges):
            return None
        distance = scan.ranges[index]
        if math.isinf(distance) or math.isnan(distance):
            return None
        return distance

    def classify_environment(self, labels):
        """Classify the environment by counting keyword matches in the detected labels."""
        env_counts = {env: 0 for env in self.ENV_KEYWORDS}
        for label in labels:
            for env, keywords in self.ENV_KEYWORDS.items():
                for kw in keywords:
                    if kw.lower() in label.lower():
                        env_counts[env] += 1
        best_env = max(env_counts, key=env_counts.get)
        return best_env if env_counts[best_env] > 0 else "unknown"

    def process_segmentation(self, cv_image):
        """
        Perform panoptic segmentation on a given OpenCV image.
        Computes bounding boxes for detected segments, estimates distance using LiDAR,
        and annotates the image with both the label and the estimated distance (plus
        local coordinates relative to the tiago-base).
        Returns the annotated image and a list of detected labels.
        """
        # Convert BGR (OpenCV) to a PIL image and resize.
        pil_img = PILImage.fromarray(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))
        pil_img = pil_img.resize(self.image_resolution, PILImage.BILINEAR)

        inputs = self.processor(images=pil_img, return_tensors="pt")
        inputs = {k: v.to("cpu") for k, v in inputs.items()}

        with torch.no_grad():
            outputs = self.model(**inputs)

        # Get panoptic segmentation results.
        result = self.processor.post_process_panoptic_segmentation(
            outputs, target_sizes=[pil_img.size[::-1]]
        )[0]
        segmentation = result["segmentation"].cpu().numpy().astype(np.int32)
        segments_info = result["segments_info"]

        # Start annotating on a copy of the image.
        annotated_img = np.array(pil_img).copy()  # RGB image for annotation
        detected_labels = []

        # Process each segment (each detected object)
        for seg in segments_info:
            seg_id = seg["id"]
            label_id = seg["label_id"]
            # Get label name from the model’s configuration.
            label_name = self.model.config.id2label.get(str(label_id), str(label_id))
            # Find the pixels for the segment.
            indices = np.where(segmentation == seg_id)
            if indices[0].size == 0 or indices[1].size == 0:
                continue
            x_min = int(indices[1].min())
            x_max = int(indices[1].max())
            y_min = int(indices[0].min())
            y_max = int(indices[0].max())
            # Calculate center x coordinate.
            cx = (x_min + x_max) // 2

            # Estimate distance using LiDAR data.
            distance = self.estimate_distance(cx, self.image_resolution[0])
            if distance is not None:
                # Compute local (x,y) relative to robot assuming camera is forward-facing.
                half_width = self.image_resolution[0] / 2.0
                angle_offset = (cx - half_width) / half_width * (self.camera_fov / 2.0)
                local_x = distance * math.cos(angle_offset)
                local_y = distance * math.sin(angle_offset)
                annotation = f"{label_name} {distance:.2f}m ({local_x:.2f},{local_y:.2f})"
            else:
                annotation = label_name

            # Draw bounding box and overlay annotation.
            cv2.rectangle(annotated_img, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
            cv2.putText(annotated_img, annotation, (x_min, max(y_min - 10, 0)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            detected_labels.append(label_name)

        return annotated_img, detected_labels

    def fuse_annotations(self, img_left, img_right):
        """Fuse the annotated images from the left and right cameras side-by-side."""
        return np.hstack((img_left, img_right))

    def image_callback(self, left_msg, right_msg):
        self.frame_count += 1
        # Process segmentation only on every Nth frame to save computation.
        if self.frame_count % self.segmentation_interval != 0:
            return

        try:
            left_cv = self.bridge.imgmsg_to_cv2(left_msg, desired_encoding='bgr8')
            right_cv = self.bridge.imgmsg_to_cv2(right_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")
            return

        self.get_logger().info("Performing panoptic segmentation...")

        # Process segmentation and get detected labels from each image.
        annotated_left, labels_left = self.process_segmentation(left_cv)
        annotated_right, labels_right = self.process_segmentation(right_cv)
        combined_labels = labels_left + labels_right

        # Classify environment based on the detected labels.
        env_type = self.classify_environment(combined_labels)

        # Fuse the two annotated images (for example, side-by-side).
        fused_image = self.fuse_annotations(annotated_left, annotated_right)

        # Overlay environment classification on the fused image.
        cv2.putText(fused_image, f"Env: {env_type}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)

        # Overlay the current robot odometry if available.
        if self.latest_odom is not None:
            pos = self.latest_odom.pose.pose.position
            odom_text = f"Odom: ({pos.x:.2f}, {pos.y:.2f})"
            cv2.putText(fused_image, odom_text, (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        # Publish the annotated fused image.
        annotated_msg = self.bridge.cv2_to_imgmsg(fused_image, encoding="bgr8")
        self.publisher.publish(annotated_msg)
        self.get_logger().info("Published annotated segmentation image.")

def main(args=None):
    rclpy.init(args=args)
    node = PanopticSegmentationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

