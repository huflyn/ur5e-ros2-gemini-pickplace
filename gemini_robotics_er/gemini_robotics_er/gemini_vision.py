#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy
import cv2
import numpy as np
from cv_bridge import CvBridge

import message_filters
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point

# Messages & Services
from color_detection_msgs.msg import LegoBrick
from color_detection_msgs.srv import DetectBricks

# Gemini API
from google import genai
from google.genai import types
from PIL import Image as PILImage
# for structured output
import json
from pydantic import BaseModel, Field
from typing import List


# ── Pydantic Models for Gemini Structured Output ───────────────────────
class BrickDetection(BaseModel):
    color: str = Field(description="Brick color, e.g. 'red', 'blue', 'green'")
    bounding_box_2d: List[int] = Field(description="[ymin, xmin, ymax, xmax] normalized 0-1000")

class DetectionResult(BaseModel):
    bricks: List[BrickDetection]

# ───────────────────────────────────────────────────────────────────────


class GeminiVisionNode(Node):
    def __init__(self):
        super().__init__('gemini_vision_node')

        # ── Parameters ─────────────────────────────────────────
        self.declare_parameter('color_image_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_image_topic', '/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('robot_base_frame', 'base_link')
        self.declare_parameter('max_depth_m', 1)

        rgb_topic = self.get_parameter('color_image_topic').value
        depth_topic = self.get_parameter('depth_image_topic').value
        info_topic = self.get_parameter('camera_info_topic').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.robot_base_frame = self.get_parameter('robot_base_frame').value
        self.max_depth = self.get_parameter('max_depth_m').value

        # ── TF2 ────────────────────────────────────────────────
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── Gemini API Setup ───────────────────────────────────
        self.api_key = os.environ.get('GEMINI_API_KEY')
        if not self.api_key:
            self.get_logger().fatal("GEMINI_API_KEY not set!")
            raise RuntimeError("Missing API key")

        self.declare_parameter('gemini_model', 'gemini-robotics-er-1.5-preview')
        self.model_id = self.get_parameter('gemini_model').value

        self.client = genai.Client(api_key=self.api_key)
        self.get_logger().info(f"Model: {self.model_id}")

        # ── Image Sync ─────────────────────────────────────────
        self.bridge = CvBridge()
        self.camera_info = None
        self.latest_rgb = None
        self.latest_depth = None

        self.rgb_sub = message_filters.Subscriber(self, Image, rgb_topic)
        self.depth_sub = message_filters.Subscriber(self, Image, depth_topic)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], queue_size=5, slop=0.1
        )
        self.ts.registerCallback(self._image_cb)
        self.info_sub = self.create_subscription(
            CameraInfo, info_topic, self._info_cb, 1
        )

        # ── Service ────────────────────────────────────────────
        self.detect_srv = self.create_service(
            DetectBricks, '/detect_bricks', self._detect_callback
        )

        # ── Publishers ─────────────────────────────────────────
        self.brick_pub = self.create_publisher(LegoBrick, '/lego_brick_info', 10)
        self.annotated_pub = self.create_publisher(Image, '/gemini_vision/annotated_image', 10)
        self.get_logger().info("GeminiVisionNode ready. Waiting for /detect_bricks calls...")

    # ── Callbacks ──────────────────────────────────────────────

    def _info_cb(self, msg):
        if self.camera_info is None:
            self.camera_info = msg
            self.get_logger().info("Camera intrinsics received.")

    def _image_cb(self, rgb_msg, depth_msg):
        self.latest_rgb = rgb_msg
        self.latest_depth = depth_msg

    # ── Service Handler ────────────────────────────────────────

    def _detect_callback(self, request, response):
        """
        Wird vom Orchestrator aufgerufen:
          ros2 service call /detect_bricks color_detection_msgs/srv/DetectBricks
        """
        self.get_logger().info("Service /detect_bricks called.")

        # Prüfe ob Sensordaten vorhanden
        if self.latest_rgb is None or self.latest_depth is None:
            response.success = False
            response.error_message = "No image data received yet"
            return response

        if self.camera_info is None:
            response.success = False
            response.error_message = "No camera_info received yet"
            return response

        # Bilder konvertieren
        cv_rgb = self.bridge.imgmsg_to_cv2(self.latest_rgb, 'rgb8')
        cv_depth = self.bridge.imgmsg_to_cv2(self.latest_depth, 'passthrough')
        pil_img = PILImage.fromarray(cv_rgb)

        # Gemini aufrufen
        bricks_data = self._call_gemini(pil_img)

        if bricks_data is None:
            response.success = False
            response.error_message = "Gemini API call failed"
            return response

        if not bricks_data:
            response.success = True
            response.error_message = ""
            response.bricks = []
            self.get_logger().info("No bricks detected.")
            return response

        # 3D-Positionen berechnen
        detected = self._process_detections(bricks_data, cv_rgb, cv_depth)

        response.success = True
        response.error_message = ""
        response.bricks = detected

        self.get_logger().info(f"Returning {len(detected)} brick(s).")
        return response

    # ── Gemini API ─────────────────────────────────────────────

    def _call_gemini(self, pil_img):
        """Sendet Bild an Gemini, gibt Liste von Brick-Dicts zurück."""

        prompt = (
            "Identify all individual LEGO bricks visible on the table surface. "
            "Ignore any boxes, containers, or non-brick objects. "
            "Return color and bounding box for each brick."
        )

        try:
            resp = self.client.models.generate_content(
                model=self.model_id,
                contents=[prompt, pil_img],
                config=types.GenerateContentConfig(
                    response_mime_type="application/json",
                    response_json_schema=DetectionResult.model_json_schema(),
                    temperature=0.1,
                    # thinking_config=types.ThinkingConfig(thinking_budget=1024)
                    # Turn off thinking:
                    thinking_config=types.ThinkingConfig(thinking_budget=0),
                    # Turn on dynamic thinking:
                    # thinking_config=types.ThinkingConfig(thinking_budget=-1)
                ),
            )
            # Gemini gibt JSON-String zurück, der in DetectionResult geparst wird
            result = DetectionResult.model_validate_json(resp.text)
            self.get_logger().info(f"Gemini response: {result.model_dump_json(indent=2)}")
            return result.bricks

        except Exception as e:
            self.get_logger().error(f"Gemini error: {e}")
            return None

    # ── 2D → 3D Projection ────────────────────────────────────

    def _process_detections(self, bricks_data, cv_rgb, cv_depth):
        img_h, img_w = cv_rgb.shape[:2]
        K = self.camera_info.k
        fx, fy, cx, cy = K[0], K[4], K[2], K[5]

        results = []

        # Work on BGR copy for annotation (cv2 drawing uses BGR)
        annotated_bgr = cv2.cvtColor(cv_rgb, cv2.COLOR_RGB2BGR)

        # Color map for drawing (BGR format)
        color_map = {
            'red':    (0, 0, 255),
            'blue':   (255, 0, 0),
            'green':  (0, 200, 0),
            'yellow': (0, 255, 255),
            'orange': (0, 165, 255),
            'white':  (255, 255, 255),
            'black':  (128, 128, 128),
        }

        for brick in bricks_data:
            color = brick.color.lower()
            ymin, xmin, ymax, xmax = brick.bounding_box_2d

            # Normalized (0-1000) to pixel coordinates
            px_xmin = int((xmin / 1000.0) * img_w)
            px_ymin = int((ymin / 1000.0) * img_h)
            px_xmax = int((xmax / 1000.0) * img_w)
            px_ymax = int((ymax / 1000.0) * img_h)

            # Center pixel
            px_cx = np.clip((px_xmin + px_xmax) // 2, 0, img_w - 1)
            px_cy = np.clip((px_ymin + px_ymax) // 2, 0, img_h - 1)

            # Draw bounding box and label
            draw_color = color_map.get(color, (0, 255, 0))
            cv2.rectangle(annotated_bgr,
                          (px_xmin, px_ymin), (px_xmax, px_ymax),
                          draw_color, 2)
            cv2.circle(annotated_bgr, (px_cx, px_cy), 4, draw_color, -1)

            # Depth (median over 7x7 region)
            region = cv_depth[
                max(0, px_cy - 3):min(img_h, px_cy + 4),
                max(0, px_cx - 3):min(img_w, px_cx + 4)
            ]
            valid = region[region > 0]
            if valid.size == 0:
                self.get_logger().warn(f"No valid depth for {color} brick")
                cv2.putText(annotated_bgr, f"{color} (no depth)",
                            (px_xmin, max(20, px_ymin - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                continue

            depth_raw = float(np.median(valid))
            z = depth_raw / 1000.0 if cv_depth.dtype == np.uint16 else depth_raw

            if z <= 0.03 or z > self.max_depth:
                self.get_logger().warn(
                    f"Skipping {color} brick: Depth z={z:.3f} is out of bounds (allowed: 0.03 to {self.max_depth})"
                )
                # Zeichne die Fehler-Tiefe auch ins Bild, damit du es in rqt siehst!
                cv2.putText(annotated_bgr, f"{color} (z={z:.1f} out of bounds)",
                            (px_xmin, max(20, px_ymin - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 2)
                continue

            # Pinhole projection
            x = (px_cx - cx) * z / fx
            y = (px_cy - cy) * z / fy

            # Point in camera frame
            pt_cam = PointStamped()
            pt_cam.header.frame_id = self.camera_frame
            pt_cam.header.stamp = self.get_clock().now().to_msg()
            pt_cam.point.x = x
            pt_cam.point.y = y
            pt_cam.point.z = z

            # Transform to robot base frame
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.robot_base_frame, self.camera_frame,
                    rclpy.time.Time(),
                    rclpy.duration.Duration(seconds=1.0)
                )
                pt_base = do_transform_point(pt_cam, tf)

                msg = LegoBrick()
                msg.color.data = color
                msg.position = pt_base
                msg.camera_distance_mm = z * 1000.0
                msg.bounding_box_px = [px_xmin, px_ymin, px_xmax, px_ymax]

                results.append(msg)
                self.brick_pub.publish(msg)

                # Add position text to annotation
                label = (f"{color} ({pt_base.point.x:.2f}, "
                         f"{pt_base.point.y:.2f})")
                cv2.putText(annotated_bgr, label,
                            (px_xmin, max(20, px_ymin - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, draw_color, 2)

                self.get_logger().info(
                    f"  {color:>8s}: X={pt_base.point.x:.3f} "
                    f"Y={pt_base.point.y:.3f} Z={pt_base.point.z:.3f}"
                )

            except Exception as e:
                self.get_logger().error(f"TF error: {e}")

        # Publish annotated image as ROS topic
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_bgr, encoding='bgr8')
            annotated_msg.header.stamp = self.get_clock().now().to_msg()
            annotated_msg.header.frame_id = self.camera_frame
            self.annotated_pub.publish(annotated_msg)
            self.get_logger().info("Annotated image published on /gemini_vision/annotated_image")
        except Exception as e:
            self.get_logger().error(f"Failed to publish annotated image: {e}")

        return results


def main(args=None):
    rclpy.init(args=args)
    node = GeminiVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()