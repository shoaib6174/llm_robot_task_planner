"""Perception node: detects colored cubes from RGBD camera feed.

Uses HSV color segmentation for reliable detection in simulation.
Publishes annotated images and detection results with 3D world positions.
"""

import json

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PointStamped
import tf2_geometry_msgs  # noqa: F401 — registers PointStamped transform


# HSV ranges for each cube color (tuned for Gazebo rendering)
COLOR_RANGES = {
    'red': [
        ((0, 120, 70), (10, 255, 255)),
        ((170, 120, 70), (180, 255, 255)),
    ],
    'green': [
        ((35, 120, 70), (85, 255, 255)),
    ],
    'blue': [
        ((100, 120, 70), (130, 255, 255)),
    ],
    'yellow': [
        ((20, 120, 70), (35, 255, 255)),
    ],
}

# BGR colors for drawing bounding boxes
DRAW_COLORS = {
    'red': (0, 0, 255),
    'green': (0, 255, 0),
    'blue': (255, 0, 0),
    'yellow': (0, 255, 255),
}

MIN_CONTOUR_AREA = 30


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        self.bridge = CvBridge()

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera/image', self.image_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth_image', self.depth_callback, 10)
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.camera_info_callback, 10)

        # Publishers
        self.detection_pub = self.create_publisher(String, '/detections', 10)
        self.annotated_pub = self.create_publisher(Image, '/detections/image', 10)

        # TF2 for projecting to world frame
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # State
        self.latest_depth = None
        self.camera_info = None

        self.get_logger().info('Perception node started (color-based detection)')

    def camera_info_callback(self, msg: CameraInfo):
        self.camera_info = msg

    def depth_callback(self, msg: Image):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().warn(f'Depth conversion failed: {e}')

    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'Image conversion failed: {e}')
            return

        detections = self.detect_cubes(cv_image)
        annotated = self.draw_detections(cv_image, detections)

        # Publish detections as JSON
        det_msg = String()
        det_msg.data = json.dumps(detections)
        self.detection_pub.publish(det_msg)

        # Publish annotated image
        try:
            ann_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            ann_msg.header = msg.header
            self.annotated_pub.publish(ann_msg)
        except Exception as e:
            self.get_logger().warn(f'Annotated image publish failed: {e}')

        if detections:
            summary = []
            for d in detections:
                if 'position_map' in d:
                    px, py, pz = d['position_map']
                    summary.append(f"{d['color']}@({px:.2f},{py:.2f})")
                else:
                    summary.append(d['color'])
            self.get_logger().info(f'Detected: {summary}', throttle_duration_sec=2.0)

    def detect_cubes(self, image: np.ndarray) -> list:
        """Detect colored cubes using HSV segmentation."""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        detections = []

        for color_name, ranges in COLOR_RANGES.items():
            # Combine masks for colors with multiple ranges (e.g. red wraps around)
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for (lower, upper) in ranges:
                partial = cv2.inRange(hsv, np.array(lower), np.array(upper))
                mask = cv2.bitwise_or(mask, partial)

            # Morphological cleanup
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                area = cv2.contourArea(contour)
                if area < MIN_CONTOUR_AREA:
                    continue

                x, y, w, h = cv2.boundingRect(contour)
                cx, cy = x + w // 2, y + h // 2

                detection = {
                    'color': color_name,
                    'bbox': [int(x), int(y), int(w), int(h)],
                    'center_px': [int(cx), int(cy)],
                    'area': int(area),
                    'confidence': 1.0,
                }

                # Add depth and 3D position if available
                if self.latest_depth is not None:
                    depth_val = self._get_depth_at(cx, cy)
                    if depth_val is not None and depth_val > 0:
                        detection['depth_m'] = round(float(depth_val), 3)
                        world_pos = self._project_to_world(cx, cy, depth_val)
                        if world_pos is not None:
                            detection['position_map'] = [
                                round(world_pos[0], 3),
                                round(world_pos[1], 3),
                                round(world_pos[2], 3),
                            ]

                detections.append(detection)

        return detections

    def _get_depth_at(self, cx: int, cy: int) -> float | None:
        """Get median depth in a small window around (cx, cy)."""
        if self.latest_depth is None:
            return None
        h, w = self.latest_depth.shape[:2]
        # 5x5 window around center
        x1 = max(0, cx - 2)
        x2 = min(w, cx + 3)
        y1 = max(0, cy - 2)
        y2 = min(h, cy + 3)
        patch = self.latest_depth[y1:y2, x1:x2]
        valid = patch[np.isfinite(patch) & (patch > 0)]
        if len(valid) == 0:
            return None
        return float(np.median(valid))

    def _project_to_world(self, cx: int, cy: int, depth: float):
        """Project pixel (cx, cy) at given depth to map frame using camera intrinsics + TF2.

        Returns (x, y, z) in map frame, or None if transform unavailable.
        """
        if self.camera_info is None:
            return None

        # Camera intrinsics from K matrix [fx, 0, cx; 0, fy, cy; 0, 0, 1]
        k = self.camera_info.k
        fx, fy = k[0], k[4]
        cx0, cy0 = k[2], k[5]

        if fx == 0 or fy == 0:
            return None

        # Project pixel to 3D point in optical frame (z-forward, x-right, y-down)
        x_cam = (cx - cx0) * depth / fx
        y_cam = (cy - cy0) * depth / fy
        z_cam = depth

        # Build a PointStamped in the camera optical frame
        pt = PointStamped()
        pt.header.frame_id = self.camera_info.header.frame_id
        pt.header.stamp = Time()  # use latest available transform
        pt.point.x = x_cam
        pt.point.y = y_cam
        pt.point.z = z_cam

        try:
            pt_map = self.tf_buffer.transform(pt, 'map', timeout=rclpy.duration.Duration(seconds=0.1))
            return (pt_map.point.x, pt_map.point.y, pt_map.point.z)
        except Exception:
            return None

    def draw_detections(self, image: np.ndarray, detections: list) -> np.ndarray:
        """Draw bounding boxes and labels on the image."""
        annotated = image.copy()
        for det in detections:
            x, y, w, h = det['bbox']
            color = DRAW_COLORS.get(det['color'], (255, 255, 255))
            cv2.rectangle(annotated, (x, y), (x + w, y + h), color, 2)

            label = f"{det['color']} cube"
            if 'position_map' in det:
                px, py, pz = det['position_map']
                label += f" ({px:.1f},{py:.1f})"
            elif 'depth_m' in det:
                label += f" {det['depth_m']:.2f}m"

            # Label background
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(annotated, (x, y - th - 6), (x + tw, y), color, -1)
            cv2.putText(annotated, label, (x, y - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

        return annotated


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
