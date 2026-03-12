#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import cv2
import numpy as np
from cv_bridge import CvBridge
import math

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, TransformStamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from tf2_geometry_msgs import do_transform_point
from image_geometry import PinholeCameraModel

# For multi-threaded execution
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

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
from typing import List, Optional


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
    box_2d: List[int] = Field(description="[ymin, xmin, ymax, xmax] normalized 0-1000")
    dropoff_point: Optional[List[int]] = Field(default=None, description="Optional: [y, x] normalized 0-1000 center coordinates for the requested drop-off location based on the user prompt.")

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

        # --- Edge Margin Parameters (Unified with color_detector) ---
        self.declare_parameter('edge_margin_x', 100)
        self.declare_parameter('edge_margin_y', 1)
        self.edge_margin_x = self.get_parameter('edge_margin_x').value
        self.edge_margin_y = self.get_parameter('edge_margin_y').value

        # --- TF2 Setup ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
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

        # --- Image Sync & Subscribers ---
        self.bridge = CvBridge()
        self.camera_model = PinholeCameraModel()
        self.camera_info_ready = False
        
        self.latest_color_msg = None
        self.latest_depth_msg = None

        self.create_subscription(CameraInfo, info_topic, self.camera_info_callback, 5)
        self.color_sub = self.create_subscription(Image, rgb_topic, self.color_callback, 5)
        self.depth_sub = self.create_subscription(Image, depth_topic, self.depth_callback, 5)

        # Visualization: Timer at 10 Hz for fluid RQT/RViz updates
        self.annotated_pub = self.create_publisher(Image, '/annotated_image', 5)
        self.vis_timer = self.create_timer(0.1, self.annotate_timer_callback)

        # State variables for continuous live annotation
        self.last_brick_annotations = []
        self.last_no_depth_annotations = []
        self.last_dropoff_annotations = []

        # --- Service Server (Multithreaded) ---
        self.srv_cb_group = MutuallyExclusiveCallbackGroup()
        self.detect_srv = self.create_service(
            DetectBricks, '/detect_bricks', self.detect_callback,
            callback_group=self.srv_cb_group
        )

        # --- Initialization Message ---
        self.get_logger().info(
            "Status:\n" +
            "="*60 + "\n" +
            "🟢 GeminiVisionNode (Service Node) ready.\n" +
            "👂 Waiting for service call from client...\n" +
            "To test functionality manually, use:\n" +
            "ros2 service call /detect_bricks brick_interfaces/srv/DetectBricks \"{custom_prompt: ''}\"\n" +
            "Or with AI instruction:\n" +
            "ros2 service call /detect_bricks brick_interfaces/srv/DetectBricks \"{custom_prompt: 'Find the nearest red brick'}\"\n" +
            "="*60
        )

    # --- Callbacks ---

    def camera_info_callback(self, msg):
        if not self.camera_info_ready:
            self.get_logger().info("Camera intrinsics received.")
            self.camera_info_ready = True
            
        self.camera_model.fromCameraInfo(msg)

    def color_callback(self, msg):
        """Just cache the latest color image."""
        self.latest_color_msg = msg

    def depth_callback(self, msg):
        """Just cache the latest depth image."""
        self.latest_depth_msg = msg

    def annotate_timer_callback(self):
        """Runs at 10 Hz — publishes live annotated image for RViz/rqt."""
        if self.latest_color_msg is None:
            return

        try:
            cv_bgr = self.bridge.imgmsg_to_cv2(self.latest_color_msg, "bgr8").copy()

            # Draw detected bricks from last service call
            for ann in self.last_brick_annotations:
                color, xmin, ymin, xmax, ymax, pt_x, pt_y = ann
                draw_brick_annotation(cv_bgr, color, xmin, ymin, xmax, ymax, pt_x, pt_y, (255, 140, 72))

            # Draw "no depth" error markers with red bounding box
            for ann in self.last_no_depth_annotations:
                color, xmin, ymin, xmax, ymax = ann
                # Red bounding box
                cv2.rectangle(cv_bgr, (xmin, ymin), (xmax, ymax), (0, 0, 0), 4)
                cv2.rectangle(cv_bgr, (xmin, ymin), (xmax, ymax), (0, 0, 255), 1)
                # Red label
                cv2.putText(cv_bgr, f"{color} (no depth)", (xmin, max(20, ymin - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 2)

            # Draw drop-off targets with bounding box
            for ann in self.last_dropoff_annotations:
                color, px_x, px_y = ann
                # Draw a 60x60 marker rectangle centered on the drop point
                half = 30
                cv2.rectangle(cv_bgr, (px_x - half, px_y - half), (px_x + half, px_y + half), (0, 0, 0), 4)
                cv2.rectangle(cv_bgr, (px_x - half, px_y - half), (px_x + half, px_y + half), (255, 140, 72), 1)
                cv2.circle(cv_bgr, (px_x, px_y), 4, (255, 140, 72), -1)
                cv2.putText(cv_bgr, f"{color} drop", (px_x + 5, px_y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 140, 72), 2)

            # Publish
            msg = self.bridge.cv2_to_imgmsg(cv_bgr, encoding="bgr8")
            msg.header = self.latest_color_msg.header
            self.annotated_pub.publish(msg)

        except Exception as e:
            self.get_logger().warn(f"Live annotation failed: {e}", throttle_duration_sec=5.0)

    # --- Service Handler ---

    def detect_callback(self, request, response):
        """Triggered by the orchestrator to perform an on-demand scan.
          ros2 service call /detect_bricks brick_interfaces/srv/DetectBricks
        """
        self.get_logger().info("Service /detect_bricks called (Gemini-Vision).")
        start_time = self.get_clock().now().to_msg().sec

        if self.latest_color_msg is None or self.latest_depth_msg is None:
            self.get_logger().error("⚠️ NO IMAGES RECEIVED! Make sure Webots is publishing BOTH color and depth topics.")
            response.success = False
            response.error_message = "No image data received yet"
            return response

        if not self.camera_info_ready:
            self.get_logger().error("⚠️ NO CAMERA INFO! Waiting for /camera_info topic.")
            response.success = False
            response.error_message = "No camera_info received yet"
            return response

        # convert image
        cv_color = self.bridge.imgmsg_to_cv2(self.latest_color_msg, 'rgb8')
        pil_img = PILImage.fromarray(cv_color)

        # --- FIX: Apply Gemini Cookbook Resizing ---
        # Resizing the image to a max width of 800px prevents Gemini's internal 
        # scaling issues and returns perfectly aligned normalized coordinates.
        orig_w, orig_h = pil_img.size
        new_w = 800
        new_h = int(new_w * orig_h / orig_w)
        pil_img_resized = pil_img.resize((new_w, new_h), PILImage.Resampling.LANCZOS)

        # Depth identically to color_detector.py (strictly to mm as float)
        if self.latest_depth_msg.encoding == '32FC1':
            depth_meters = self.bridge.imgmsg_to_cv2(self.latest_depth_msg, desired_encoding="32FC1")
            cv_depth = depth_meters * 1000.0
        else:
            cv_depth = self.bridge.imgmsg_to_cv2(self.latest_depth_msg, desired_encoding="16UC1")

        # Call Gemini API
        bricks_data = self._call_gemini(pil_img_resized, request.custom_prompt)

        # If API fails or finds nothing, clear annotations (Timer will draw empty image with safe zone)
        if bricks_data is None:
            response.success = False
            response.error_message = "Gemini API call failed"
            self.last_brick_annotations = []
            self.last_no_depth_annotations = []
            self.last_dropoff_annotations = []
            return response

        if bricks_data:
            detected = self._process_detections(bricks_data, cv_color, cv_depth)
        else:
            detected = []
            self.get_logger().info("No bricks detected.")
            # Clear annotations when no bricks found
            self.last_brick_annotations = []
            self.last_no_depth_annotations = []
            self.last_dropoff_annotations = []

        response.success = True
        response.error_message = ""
        response.bricks = detected

        self.get_logger().info(f"Returning {len(detected)} brick(s). Service call complete.")
        self.get_logger().info(f"Total processing time: {self.get_clock().now().to_msg().sec - start_time} seconds")

        self.get_logger().info(
            "Status:\n" + "="*60 + "\n" +
            "🟢 GeminiVisionNode ready. Waiting for new /detect_bricks call...\n" +
            "="*60
        )

        return response

    # --- Gemini API ---

    def _call_gemini(self, pil_img, custom_prompt: Optional[str] = None) -> Optional[List[BrickDetection]]:
        """ Sends the image to Gemini, returns list of BrickDetection objects. """
        
        # System Instruction (Outlines the task for Gemini, can be extended with user prompt)
        base_prompt = (
            """Detect all Lego Bricks in the image.
            Return bounding boxes as a JSON array with labels. Never return masks or
            code fencing. Limit to 25 objects. Include as many objects as you can
            identify on the table.
            The format should be as follows:
            [{"label": "<color>", "box_2d": [ymin, xmin, ymax, xmax]}]
            normalized to 0-1000. The values in box_2d must only be integers."""
        )

        if custom_prompt:
            base_prompt += (
                f"\n\n--- USER INSTRUCTION ---\n{custom_prompt}\n\n"
                "ACT AS A ROBOTIC TASK PLANNER. Follow these rules strictly:\n"
                "1. SELECTION: ONLY return bricks that match the user's instruction.\n"
                "2. SEQUENCE: Order the array in the pick sequence.\n"
                "3. DROP-OFF: If the user specifies a target location, add a 'dropoff_point' field "
                "with the exact [y, x] normalized 0-1000 center point of the target area."
            )

        self.get_logger().info(f"Prompt: {base_prompt}")
        
        try:
            response = self.client.models.generate_content(
                model=self.model_id,
                contents=[base_prompt, pil_img],
                config=types.GenerateContentConfig(
                    response_mime_type="application/json",
                    response_json_schema=DetectionResult.model_json_schema(),
                    temperature=0.5,
                    thinking_config=types.ThinkingConfig(include_thoughts=True, thinking_budget=0), # -1 = dynamic thinking (default), 0 = disable thinking, Range: 0 to 24576
                ),
            )

            for part in response.candidates[0].content.parts:
                if not part.text:
                    continue
                if part.thought:
                    self.get_logger().info(
                        "="*60 + "\n" +
                        "Thought summary:\n" +
                        f"{part.text}"
                    )
                else:
                    self.get_logger().info(
                        "="*60 + "\n" +
                        "Answer:\n" +
                        f"{part.text}"
                    )

            # Gemini returns a JSON string that is parsed into DetectionResult
            detection_result = DetectionResult.model_validate_json(response.text)
            return detection_result.bricks

        except Exception as e:
            self.get_logger().error(f"Gemini error: {e}")
            return None

    # --- 2D → 3D Projection ---

    def _process_detections(self, bricks_data, cv_color, cv_depth):
        img_h, img_w = cv_color.shape[:2]

        results = []
        new_brick_annotations = []
        new_no_depth_annotations = []
        new_dropoff_annotations = []
        brick_counter = {}

        for brick in bricks_data:
            color = brick.color.lower()
            ymin, xmin, ymax, xmax = brick.box_2d

            # Normalized (0-1000) to pixel coordinates
            px_xmin = int((xmin / 1000.0) * img_w)
            px_ymin = int((ymin / 1000.0) * img_h)
            px_xmax = int((xmax / 1000.0) * img_w)
            px_ymax = int((ymax / 1000.0) * img_h)

            # Ensure coordinates stay within image boundaries
            px_xmin = np.clip(px_xmin, 0, img_w - 1)
            px_xmax = np.clip(px_xmax, 0, img_w - 1)
            px_ymin = np.clip(px_ymin, 0, img_h - 1)
            px_ymax = np.clip(px_ymax, 0, img_h - 1)

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

            # --- ROBUST MEDIAN DEPTH CALCULATION ---
            # Depth (median over robust 5x5 region to avoid NaN holes)
            region = cv_depth[
                max(0, px_cy - 2):min(img_h, px_cy + 3),
                max(0, px_cx - 2):min(img_w, px_cx + 3)
            ]

            # Filter out 0 and inf/NaN values
            valid_depth = region[(region > 0) & np.isfinite(region)]

            if valid_depth.size == 0:
                self.get_logger().warn(f"No valid_depth depth for {color} brick")
                new_no_depth_annotations.append((color, px_xmin, px_ymin, px_xmax, px_ymax))
                continue

            # Already in mm due to new conversion above
            depth_mm = float(np.median(valid_depth))
            z = depth_mm / 1000.0 # Convert back to meters for 3D projection

            if z <= 0.03 or z > self.max_depth:
                self.get_logger().warn(
                    f"Skipping {color} brick: Depth z={z:.3f} is out of bounds (allowed: 0.03 to {self.max_depth})"
                )
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
                msg.has_dynamic_dropoff = False

                # --- Custom Drop-off ---
                if brick.dropoff_point and len(brick.dropoff_point) == 2:
                    drop_y_norm, drop_x_norm = brick.dropoff_point
                    px_drop_x = np.clip(int((drop_x_norm / 1000.0) * img_w), 0, img_w - 1)
                    px_drop_y = np.clip(int((drop_y_norm / 1000.0) * img_h), 0, img_h - 1)

                    # Get depth for drop-off location 
                    drop_region = cv_depth[
                        max(0, px_drop_y - 5):min(img_h, px_drop_y + 6),
                        max(0, px_drop_x - 5):min(img_w, px_drop_x + 6)
                    ]
                    valid_drop_depth = drop_region[(drop_region > 0) & np.isfinite(drop_region)]
                    
                    if valid_drop_depth.size > 0:
                        drop_depth_mm = float(np.median(valid_drop_depth))
                        drop_z = drop_depth_mm / 1000.0
                        drop_ray = self.camera_model.projectPixelTo3dRay((px_drop_x, px_drop_y))
                        # Project to 3D and transform
                        drop_pt_cam = PointStamped()
                        drop_pt_cam.header.frame_id = self.camera_frame
                        drop_pt_cam.header.stamp = self.get_clock().now().to_msg()
                        drop_pt_cam.point.x = drop_ray[0] * drop_z
                        drop_pt_cam.point.y = drop_ray[1] * drop_z
                        drop_pt_cam.point.z = drop_z
                        
                        try:
                            # Transform to base_link
                            drop_pt_base = do_transform_point(drop_pt_cam, tf)
                            msg.has_dynamic_dropoff = True
                            msg.dynamic_dropoff_position = drop_pt_base.point
                            self.get_logger().info(f"Target custom Drop-off found at X={drop_pt_base.point.x:.3f}, Y={drop_pt_base.point.y:.3f}")
                            
                            # Append to live annotation list instead of drawing directly
                            new_dropoff_annotations.append((color, px_drop_x, px_drop_y))

                        except Exception as e:
                            self.get_logger().error(f"TF error for custom drop-off: {e}")
                    else:
                        self.get_logger().warn(f"No depth for custom drop-off of {color} brick")

                results.append(msg)

                # --------------------------------------------------

                # --- TF Broadcast ---
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

                # Append to live annotation list instead of drawing directly
                new_brick_annotations.append((color, px_xmin, px_ymin, px_xmax, px_ymax, pt_base.point.x, pt_base.point.y))

                self.get_logger().info(f"{color:>8s}: X={pt_base.point.x:.3f} Y={pt_base.point.y:.3f} Z={pt_base.point.z:.3f}")

            except Exception as e:
                self.get_logger().error(f"TF error: {e}")

        # Update live stream arrays
        self.last_brick_annotations = new_brick_annotations
        self.last_no_depth_annotations = new_no_depth_annotations
        self.last_dropoff_annotations = new_dropoff_annotations

        return results


def main(args=None):
    rclpy.init(args=args)
    node = GeminiVisionNode()

    # Use MultiThreadedExecutor instead of standard spin to handle concurrent callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()