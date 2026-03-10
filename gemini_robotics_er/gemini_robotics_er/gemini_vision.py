#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy
import cv2
import numpy as np
from cv_bridge import CvBridge
import math

import message_filters
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, TransformStamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from tf2_geometry_msgs import do_transform_point
from image_geometry import PinholeCameraModel

# Messages & Services
from brick_interfaces.msg import LegoBrick
from brick_interfaces.srv import DetectBricks

# Gemini API
from google import genai
from google.genai import types
from PIL import Image as PILImage

# for structured output
import json
from pydantic import BaseModel, Field
from typing import List


def draw_brick_annotation(cv_image, color_name, xmin, ymin, xmax, ymax, pt_x, pt_y, draw_color=(0, 255, 0)):
    """
    Draws a bounding box, center point, and multi-line 3D coordinate text 
    with high-contrast outlines for a detected brick.
    
    cv_image: The OpenCV image to draw on (BGR format).
    color_name: The name of the brick color (e.g., "red").
    xmin, ymin, xmax, ymax: Pixel coordinates of the bounding box.
    pt_x, pt_y: The 3D coordinates of the brick (for annotation).
    draw_color: The color to use for drawing (default is green).
    """
    center_x = (xmin + xmax) // 2
    center_y = (ymin + ymax) // 2

    # --- Draw Bounding Box and Center with Outline ---
    cv2.rectangle(cv_image, (xmin, ymin), (xmax, ymax), (0, 0, 0), 4)
    cv2.rectangle(cv_image, (xmin, ymin), (xmax, ymax), draw_color, 1)

    cv2.circle(cv_image, (center_x, center_y), 4, (0, 0, 0), -1)
    cv2.circle(cv_image, (center_x, center_y), 2, draw_color, -1)

    # --- Text Annotation ---
    FONT_SCALE_COLOR = 0.45  
    FONT_SCALE_NUMBER = 0.35
    LINE_SPACING_PX = 15    
    
    label_line1 = color_name 
    label_line2 = f"({pt_x:.2f}, {pt_y:.2f})" 
    
    # Calculate safe Y base position
    safe_top_margin = int(LINE_SPACING_PX + 25)
    y_base = max(safe_top_margin, ymin - 6) 
    
    pos_line1 = (xmin, y_base - LINE_SPACING_PX) 
    pos_line2 = (xmin, y_base)                 
    
    # Draw Line 1 (Color)
    cv2.putText(cv_image, label_line1, pos_line1, cv2.FONT_HERSHEY_SIMPLEX, FONT_SCALE_COLOR, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(cv_image, label_line1, pos_line1, cv2.FONT_HERSHEY_SIMPLEX, FONT_SCALE_COLOR, draw_color, 1, cv2.LINE_AA)
    
    # Draw Line 2 (Coordinates)
    cv2.putText(cv_image, label_line2, pos_line2, cv2.FONT_HERSHEY_SIMPLEX, FONT_SCALE_NUMBER, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(cv_image, label_line2, pos_line2, cv2.FONT_HERSHEY_SIMPLEX, FONT_SCALE_NUMBER, draw_color, 1, cv2.LINE_AA)


# ── Pydantic Models for Gemini Structured Output ───────────────────────
class BrickDetection(BaseModel):
    color: str = Field(description="Brick color, e.g. 'red', 'blue', 'green'")
    bounding_box_2d: List[int] = Field(description="[ymin, xmin, ymax, xmax] normalized 0-1000")

class DetectionResult(BaseModel):
    bricks: List[BrickDetection]

class GeminiVisionNode(Node):
    def __init__(self):
        super().__init__('gemini_vision_node')

        # --- Parameters ---
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

        # --- TF2 Setup ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- TF2 Broadcaster ---
        # Used to publish the 3D position of the bricks to RViz
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- Gemini API Setup ---
        self.api_key = os.environ.get('GEMINI_API_KEY')
        if not self.api_key:
            self.get_logger().fatal("GEMINI_API_KEY not set!")
            raise RuntimeError("Missing API key")

        self.declare_parameter('gemini_model', 'gemini-robotics-er-1.5-preview')
        self.model_id = self.get_parameter('gemini_model').value

        self.client = genai.Client(api_key=self.api_key)
        self.get_logger().info(f"Model: {self.model_id}")

        # --- Image Sync ---
        self.bridge = CvBridge()
        self.camera_model = PinholeCameraModel()
        self.camera_info_ready = False
        self.latest_rgb = None
        self.latest_depth = None

        self.rgb_sub = message_filters.Subscriber(self, Image, rgb_topic)
        self.depth_sub = message_filters.Subscriber(self, Image, depth_topic)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], queue_size=5, slop=0.1
        )
        self.ts.registerCallback(self._image_cb)
        self.info_sub = self.create_subscription(
            CameraInfo, info_topic, self._camera_info_cb, 1
        )

        # --- Service Server ---
        self.detect_srv = self.create_service(
            DetectBricks, '/detect_bricks', self._detect_callback
        )

        # --- Visualization Publisher ---
        self.latest_annotated_msg = None # Stores the last processed image
        self.annotated_pub = self.create_publisher(Image, '/annotated_image', 10) # Publisher for the annotated image topic
        PUB_HZ = 5 # Frequency in Hertz for the visualization updates
        self.vis_timer = self.create_timer(1/PUB_HZ, self.publish_vis_timer) # Timer that triggers the visualization publishing at the specified rate

        # --- Initialization Message ---
        self.get_logger().info(
            "Status:"
            "\n" + "="*60 + "\n" +
            "🟢 GeminiVisionNode (Service Node) ready.\n" +
            "👂 Waiting for service call from client...\n" +
            "To test functionality, you can call the service in a new terminal:\n" +
            "ros2 service call /detect_bricks brick_interfaces/srv/DetectBricks\n" +
            "="*60
        )

    # --- Callbacks ---

    def _camera_info_cb(self, msg):
        # Print the log only once when the first message arrives
        if not self.camera_info_ready:
            self.get_logger().info("Camera intrinsics received.")
            self.camera_info_ready = True
            
        # Safely feed the data into the model
        self.camera_model.fromCameraInfo(msg)

    def _image_cb(self, rgb_msg, depth_msg):
        self.latest_rgb = rgb_msg
        self.latest_depth = depth_msg

    def publish_vis_timer(self):
        """Continuously publishes the last annotated image for rqt_image_view."""
        if self.latest_annotated_msg is not None:
            # Update the timestamp to prevent RQT from dropping "old" messages
            self.latest_annotated_msg.header.stamp = self.get_clock().now().to_msg()
            self.annotated_pub.publish(self.latest_annotated_msg)
    
    def _publish_annotated(self, cv_rgb):
        """Publishes an image (RGB input) for rqt, even without detections."""
        try:
            annotated_bgr = cv2.cvtColor(cv_rgb, cv2.COLOR_RGB2BGR)
            msg = self.bridge.cv2_to_imgmsg(annotated_bgr, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.camera_frame
            self.latest_annotated_msg = msg
            self.annotated_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish image: {e}")

    # --- Service Handler ---

    def _detect_callback(self, request, response):
        """Triggered by the orchestrator to perform an on-demand scan.
          ros2 service call /detect_bricks brick_interfaces/srv/DetectBricks
        """
        self.get_logger().info("Service /detect_bricks called (Gemini-Vision).")

        if self.latest_rgb is None or self.latest_depth is None:
            response.success = False
            response.error_message = "No image data received yet"
            return response

        if not self.camera_info_ready:
            response.success = False
            response.error_message = "No camera_info received yet"
            return response

        # Bilder konvertieren
        cv_color = self.bridge.imgmsg_to_cv2(self.latest_rgb, 'rgb8')
        cv_depth = self.bridge.imgmsg_to_cv2(self.latest_depth, 'passthrough')
        pil_img = PILImage.fromarray(cv_color)

        # Gemini aufrufen
        bricks_data = self._call_gemini(pil_img)

        if bricks_data is None:
            response.success = False
            response.error_message = "Gemini API call failed"
            self._publish_annotated(cv_color)
            return response

        if bricks_data:
            detected = self._process_detections(bricks_data, cv_color, cv_depth)
        else:
            detected = []
            self.get_logger().info("No bricks detected.")
            self._publish_annotated(cv_color)

        response.success = True
        response.error_message = ""
        response.bricks = detected

        self.get_logger().info(f"Returning {len(detected)} brick(s). Service call complete.")
        return response

    # --- Gemini API ---

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
                    temperature=1.0,
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

    # --- 2D → 3D Projection ---

    def _process_detections(self, bricks_data, cv_color, cv_depth):
        img_h, img_w = cv_color.shape[:2]

        results = []

        # Work on BGR copy for annotation (cv2 drawing uses BGR)
        annotated_bgr = cv2.cvtColor(cv_color, cv2.COLOR_RGB2BGR)

        # Dictionary to track counts of each color for unique TF names
        brick_counter = {}

        for brick in bricks_data:
            color = brick.color.lower()
            ymin, xmin, ymax, xmax = brick.bounding_box_2d

            # Normalized (0-1000) to pixel coordinates
            px_xmin = int((xmin / 1000.0) * img_w)
            px_ymin = int((ymin / 1000.0) * img_h)
            px_xmax = int((xmax / 1000.0) * img_w)
            px_ymax = int((ymax / 1000.0) * img_h)

            # Evaluate aspect ratio to determine orientation (yaw)
            w = px_xmax - px_xmin
            h = px_ymax - px_ymin

            # Calculate aspect ratio based on horizontal camera perspective
            aspect_ratio = w / h if h > 0 else 0.0

            # Orientation logic for 4x2 bricks from side-view:
            if aspect_ratio >= 1.41:
                yaw_degrees = 0.0 # Brick is horizontal (parallel to camera plane)
            elif aspect_ratio > 0:
                yaw_degrees = 30.0 # Brick is vertical (pointing away from camera)
            else:
                yaw_degrees = 0.0 # Fallback for invalid_depth detections

            # Center pixel
            px_cx = np.clip((px_xmin + px_xmax) // 2, 0, img_w - 1)
            px_cy = np.clip((px_ymin + px_ymax) // 2, 0, img_h - 1)

            # Depth (median over 7x7 region)
            region = cv_depth[
                max(0, px_cy - 3):min(img_h, px_cy + 4),
                max(0, px_cx - 3):min(img_w, px_cx + 4)
            ]

            # Filter out 0 and inf/NaN values
            valid_depth = region[(region > 0) & np.isfinite(region)]

            if valid_depth.size == 0:
                self.get_logger().warn(f"No valid_depth depth for {color} brick")
                cv2.putText(annotated_bgr, f"{color} (no depth)",
                            (px_xmin, max(20, px_ymin - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                continue

            depth_raw = float(np.median(valid_depth))
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

            # Use the PinholeCameraModel to get the 3D ray, then scale by depth (z)
            ray = self.camera_model.projectPixelTo3dRay((px_cx, px_cy))
            x = ray[0] * z
            y = ray[1] * z

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
                msg.yaw_degrees = yaw_degrees
                msg.bounding_box_px = [px_xmin, px_ymin, px_xmax, px_ymax]

                results.append(msg)

                # --- Broadcast TF Frame for RViz ---
                # Get the current count for this color and increment
                current_count = brick_counter.get(color, 0)
                brick_counter[color] = current_count + 1
                
                # Create a unique child frame ID, e.g., "brick_red_0"
                child_frame = f"brick_{color}_{current_count}"
                
                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = self.robot_base_frame  # Usually 'base_link'
                t.child_frame_id = child_frame
                
                # Apply the 3D translation
                t.transform.translation.x = pt_base.point.x
                t.transform.translation.y = pt_base.point.y
                t.transform.translation.z = pt_base.point.z
                
                # Convert yaw from degrees to radians, then to a quaternion (Z-axis rotation)
                yaw_rad = math.radians(yaw_degrees)
                t.transform.rotation.x = 0.0
                t.transform.rotation.y = 0.0
                t.transform.rotation.z = math.sin(yaw_rad / 2.0)
                t.transform.rotation.w = math.cos(yaw_rad / 2.0)
                
                # Publish the transform to the ROS 2 tf tree
                self.tf_broadcaster.sendTransform(t)
                # ------------------------------------

                # --- Annotation ---
                draw_color = (255, 140, 72)
                draw_brick_annotation(annotated_bgr, color, px_xmin, px_ymin, px_xmax, px_ymax, pt_base.point.x, pt_base.point.y, draw_color)

                self.get_logger().info(f"  {color:>8s}: X={pt_base.point.x:.3f} " f"Y={pt_base.point.y:.3f} Z={pt_base.point.z:.3f}")

            except Exception as e:
                self.get_logger().error(f"TF error: {e}")

        # Publish annotated image as ROS topic
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_bgr, encoding='bgr8')
            annotated_msg.header.stamp = self.get_clock().now().to_msg()
            annotated_msg.header.frame_id = self.camera_frame
            
            # --- Save the message for the continuous timer ---
            self.latest_annotated_msg = annotated_msg
            
            self.annotated_pub.publish(annotated_msg)
            self.get_logger().info("Annotated image published on /annotated_image")
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