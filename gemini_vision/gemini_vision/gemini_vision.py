#!/usr/bin/env python3

''' 
A vision system for a robotic pick-and-place task using the Gemini API.
Input: RGB image. Output: Detected objects with bounding boxes, colors, and potential drop-off locations based on user instructions (optional).
The node subscribes to camera topics, processes images with Gemini, and returns 3D coordinates for detected objects.
'''

import os
import cv2
import numpy as np
from cv_bridge import CvBridge
import textwrap
import time
import io
import random
import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from tf2_geometry_msgs import do_transform_point
from image_geometry import PinholeCameraModel
from geometry_msgs.msg import PointStamped, TransformStamped

from object_interfaces.msg import DetectedObject
from object_interfaces.srv import DetectObjects

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from threading import Lock

from google import genai
from google.genai import types
from PIL import Image as PILImage
from PIL import ImageColor
from pydantic import BaseModel, Field
from typing import List, Optional


# --- GEMINI CONFIGURATION ---

GEMINI_MODELS = [
    "gemini-3-flash-preview",
    "gemini-3.1-flash-lite-preview",
    "gemini-robotics-er-1.5-preview"
]
GEMINI_MODEL = GEMINI_MODELS[1] # Set your default model here

GEMINI_THINKING_LEVELS = ["minimal", "low", "medium", "high"] # Gemini 3 Flash default: high / Gemini 3.1 Flash-Lite default: minimal
GEMINI_THINKING_LEVEL_DEFAULT = GEMINI_THINKING_LEVELS[2] # Change this to switch thinking levels, for complex user prompts medium (2) or high (3) is recommended, for the simple default prompt minimal (0) works fine and is faster (using Gemini 3.1 Flash-Lite Preview)
GEMINI_THINKING_LEVEL_3FLASH = GEMINI_THINKING_LEVELS[3] # Dedicated thinking level for Gemini 3 Flash
GEMINI_THINKING_LEVEL_31FLASHLITE = GEMINI_THINKING_LEVELS[2] # Dedicated thinking level for Gemini 3.1 Flash-Lite


GEMINI_THINKING_BUDGET = -1 # Gemini Robotics-ER 1.5 uses thinking_budget (default: -1 for dynamic thinking) instead of thinking_level, set to 0 for no thinking, range: 0 to 24576

GEMINI_DEFAULT_PROMPT = textwrap.dedent("""\
    Detect all bricks on the table
""")

GEMINI_SYSTEM_PROMPT = textwrap.dedent("""\
    You are a vision system for a robotic pick-and-place task.

    SCENE CONTEXT:
    The camera is placed directly on the table in front of the robot. The camera's field of view is perfectly horizontal and parallel to the table surface. 
    Because of this perspective, the flat table surface is only visible in the lower portion of the image. Objects that appear higher up in the image are most likely further away in the background or off the table entirely.

    Your job is to analyze the image, detect building blocks (and other requested objects), and determine their bounding boxes, colors, and potential drop-off locations based on user instructions.

    Return a JSON list matching this exact schema:
    {
        "objects": [
        {
            "object_name": "<name>",
            "color": "<color>",
            "box_2d": [ymin, xmin, ymax, xmax],
            "user_dropoff": boolean,
            "dropoff_point_2d": [y, x] | null,
            "dropoff_coords_m": [x, y] | null
        }
      ]
    }

    JSON Field Instructions:
    1. object_name: The name of the object. STRICTLY use only a single word (e.g., "brick", "pen", "screw", "car", "candy", etc.).
    2. color: The color of the object (e.g., "red", "blue", "green", etc.).
    3. box_2d: Bounding box of the object. Normalized integers 0-1000.
    4. user_dropoff: Evaluate this STRICTLY PER INDIVIDUAL OBJECT.
       - Set to true if the user specifies ANY physical destination, if you can determine it. This includes VISUAL TARGETS (e.g., "on the colored areas", "onto the matching fields", "next to the blue object") OR explicit METRIC COORDINATES (e.g., "put it at x=0.5, y=0.1").
       - Set to false ONLY if the user gives a general command WITHOUT any destination at all (e.g., "pick all red bricks", "sort the blocks by color").
    5. dropoff_point_2d: 
       - If user_dropoff is true AND the target is visual, return the dropoff point [y, x] in normalized integers 0-1000.
       - If the target is strictly metric coordinates, or if user_dropoff is false, this MUST be null.
       CRITICAL CONSTRAINT FOR VISUAL DROPOFFS: The drop-off point MUST be located strictly on the flat wooden table surface visible at the bottom of the image. 
       - NEVER place the drop-off point on the background, walls, windows, or cabinets.
       - Look at the y-coordinates of the detected objects resting on the table. Your drop-off y-coordinate must fall within that same horizontal band (typically y > 700).
       - COLLISION AVOIDANCE: Ensure the drop-off point is NOT located on top of or too close to OTHER currently unpicked objects on the table. Find a clear, empty spot within the target area to avoid blocking future picks.
       - GROUPING RULE: If multiple objects belong to the exact same visual target area, use the EXACT SAME center point [y, x] for all of them. Do NOT calculate spatial offsets to place them side-by-side UNLESS the user explicitly instructs you to do so.
    6. dropoff_coords_m:
       - If user_dropoff is true AND the user provided explicit numerical metric coordinates in the prompt (e.g., "x=0.4, y=0.5"), return them here as [x, y] floats in meters. 
       - Otherwise, this MUST be null.

    General Instructions:
    - ONLY SINGLE OBJECTS, no builds or groups or stacks.
    - SELECTION: Only return objects that match the user's instructions.
    - SEQUENCE: Order the array logically by pick sequence (e.g., top-most or most relevant objects first).
    - LIMIT: Maximum 15 objects per image.
""")


# --- PYDANTIC MODELS ---

class ObjectDetection(BaseModel):
    object_name: str = Field(description="Name/type of the object (e.g., 'building object', 'pen')")
    label: str = Field(description="Dominant color of the object (e.g., 'red', 'blue')")
    box_2d: List[int] = Field(description="[ymin, xmin, ymax, xmax] normalized 0-1000")
    user_dropoff: bool = Field(
        default=False, 
        description="True ONLY if the user explicitly specified a custom drop-off location or target for this object. Otherwise False."
    )
    dropoff_point_2d: Optional[List[int]] = Field(
        default=None, 
        description="[y, x] normalized 0-1000. Use ONLY if the custom drop-off is a visual feature visible in the image."
    )
    dropoff_coords_m: Optional[List[float]] = Field(
        default=None, 
        description="[x, y] coordinates in meters. Use ONLY if the user explicitly provided numerical metric coordinates in the prompt."
    )
    # The following fields are not part of the Gemini response but will be filled in later for internal use (excluded from JSON schema)
    position_3d: Optional[List[float]] = Field(default=None, exclude=True)
    dropoff_3d: Optional[List[float]] = Field(default=None, exclude=True)
    yaw_degrees: float = Field(default=0.0, exclude=True)

class DetectionResult(BaseModel):
    objects: List[ObjectDetection]


#  --- HELPER FUNCTIONS ---

def build_prompt(user_prompt: Optional[str] = None) -> str:
    """Builds the Gemini prompt."""
    prompt = GEMINI_DEFAULT_PROMPT

    if user_prompt:
        prompt = textwrap.dedent(f"""\
            {user_prompt}""")

    return prompt


def normalized_to_pixel(coords: List[int], img_w: int, img_h: int) -> tuple:
    """
    Converts normalized coordinates (0-1000) to absolute pixel coordinates.
    Handles both bounding boxes (4 items) and points (2 items).
    """
    if len(coords) == 4:
        # Bounding Box: [ymin, xmin, ymax, xmax]
        ymin_n, xmin_n, ymax_n, xmax_n = coords

        # Calculate raw pixels
        y1 = int(ymin_n / 1000.0 * img_h)
        x1 = int(xmin_n / 1000.0 * img_w)
        y2 = int(ymax_n / 1000.0 * img_h)
        x2 = int(xmax_n / 1000.0 * img_w)
        
        # Ensure correct order (ymin, xmin, ymax, xmax)
        return (
            min(y1, y2),
            min(x1, x2),
            max(y1, y2),
            max(x1, x2)
        )

    elif len(coords) == 2:
        # Point: [y, x]
        y_n, x_n = coords
        return (
            int(y_n / 1000.0 * img_h),
            int(x_n / 1000.0 * img_w)
        )

    else:
        raise ValueError(f"Expected 2 or 4 coordinates, got {len(coords)}")


def get_robust_depth(coords_2d: List[int], cv_depth: np.ndarray, 
                     img_w: int, img_h: int) -> Optional[float]:
    """
    Extracts median depth from a region around the center.
    Adaptive region size based on bounding box dimensions.
    """
    if len(coords_2d) == 4:
        ymin, xmin, ymax, xmax = normalized_to_pixel(coords_2d, img_w, img_h)
        # ✅ FIX: Division durch 2, nicht 4!
        cx = (xmin + xmax) // 2
        cy = (ymin + ymax) // 2

        # ✅ Adaptive Region: 30% der BBox-Größe, aber min 3, max 20
        box_w = xmax - xmin
        box_h = ymax - ymin
        half_w = max(3, min(20, int(box_w * 0.15)))
        half_h = max(3, min(20, int(box_h * 0.15)))

    elif len(coords_2d) == 2:
        cy, cx = normalized_to_pixel(coords_2d, img_w, img_h)
        half_w = 5
        half_h = 5
    else:
        return None

    # Clamp to image bounds
    cx = np.clip(cx, 0, img_w - 1)
    cy = np.clip(cy, 0, img_h - 1)

    # Extract region
    region = cv_depth[
        max(0, cy - half_h):min(img_h, cy + half_h + 1),
        max(0, cx - half_w):min(img_w, cx + half_w + 1)
    ]

    # Filter invalid values
    valid_depth = region[(region > 0) & np.isfinite(region)]

    if valid_depth.size == 0:
        return None

    return float(np.median(valid_depth))

def get_robust_depth_multi_frame(self, coords_2d, img_w, img_h, 
                                  num_frames=5, delay=0.05):
    """Sammelt Depth über mehrere Frames für Zuverlässigkeit."""
    depths = []
    for _ in range(num_frames):
        if self.latest_depth_msg is None:
            continue

        if self.latest_depth_msg.encoding == '32FC1':
            cv_depth = self.bridge.imgmsg_to_cv2(
                self.latest_depth_msg, "32FC1") * 1000.0
        else:
            cv_depth = self.bridge.imgmsg_to_cv2(
                self.latest_depth_msg, "16UC1").astype(np.float32)

        d = get_robust_depth(coords_2d, cv_depth, img_w, img_h)
        if d is not None:
            depths.append(d)
        time.sleep(delay)

    if not depths:
        return None

    return float(np.median(depths))


def draw_bbox(image, label, ymin, xmin, ymax, xmax, pt_3d=None, color=(255, 140, 72)):
    """Draws a bounding box with label and optional 3D coordinates on the image."""
    # Bounding Box: Black outline (thick), Colored inner line (thin)
    cv2.rectangle(image, (xmin, ymin), (xmax, ymax), (0, 0, 0), 3)
    cv2.rectangle(image, (xmin, ymin), (xmax, ymax), color, 1)

    # Center point
    cx, cy = (xmin + xmax) // 2, (ymin + ymax) // 2
    cv2.circle(image, (cx, cy), 4, (0, 0, 0), -1)
    cv2.circle(image, (cx, cy), 2, color, -1)

    # Text styling: White outline, Black inner text
    text_outline = (255, 255, 255)
    text_color = (0, 0, 0)

    # Label text
    pos1 = (xmin, max(35, ymin - 20))
    cv2.putText(image, label, pos1, cv2.FONT_HERSHEY_SIMPLEX, 0.45, text_outline, 3, cv2.LINE_AA)
    cv2.putText(image, label, pos1, cv2.FONT_HERSHEY_SIMPLEX, 0.45, text_color, 1, cv2.LINE_AA)

    # 3D Coordinates text
    if pt_3d:
        coord_text = f"X:{pt_3d[0]:.2f} Y:{pt_3d[1]:.2f}"
        pos2 = (xmin, max(20, ymin - 5))
        cv2.putText(image, coord_text, pos2, cv2.FONT_HERSHEY_SIMPLEX, 0.35, text_outline, 3, cv2.LINE_AA)
        cv2.putText(image, coord_text, pos2, cv2.FONT_HERSHEY_SIMPLEX, 0.35, text_color, 1, cv2.LINE_AA)
    else:
        # Fallback if depth is missing
        pos2 = (xmin, max(20, ymin - 5))
        cv2.putText(image, "(no depth)", pos2, cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 2, cv2.LINE_AA)


def draw_point(image, label, y, x, color=(72, 255, 140)):
    """Draws a target point marker with a label on the image."""
    # Marker: Black outline (thick), Colored inner line (thin)
    cv2.drawMarker(image, (x, y), (0, 0, 0), markerType=cv2.MARKER_CROSS, markerSize=20, thickness=3)
    cv2.drawMarker(image, (x, y), color, markerType=cv2.MARKER_CROSS, markerSize=20, thickness=1)

    # Text styling: White outline, Black inner text
    text_outline = (255, 255, 255)
    text_color = (0, 0, 0)

    # Draw text label slightly offset from the point
    pos = (x + 10, y - 10)
    cv2.putText(image, label, pos, cv2.FONT_HERSHEY_SIMPLEX, 0.45, text_outline, 3, cv2.LINE_AA)
    cv2.putText(image, label, pos, cv2.FONT_HERSHEY_SIMPLEX, 0.45, text_color, 1, cv2.LINE_AA)


def draw_all_annotations(image: np.ndarray, detections: List[ObjectDetection], img_w: int, img_h: int) -> np.ndarray:
    """Draws all bounding boxes and dropoff points on the image."""
    for object in detections:
        object_color = get_color_for_label(object.label)
        display_text = f"{object.label} {object.object_name}"

        # 1. Draw Object Box (jetzt mit 3D-Koordinatenübergabe)
        ymin, xmin, ymax, xmax = normalized_to_pixel(object.box_2d, img_w, img_h)
        draw_bbox(image, display_text, ymin, xmin, ymax, xmax, pt_3d=object.position_3d, color=object_color)

        # 2. Draw Dropoff Point
        if object.dropoff_point_2d is not None:
            py, px = normalized_to_pixel(object.dropoff_point_2d, img_w, img_h)
            draw_point(image, f"{display_text} dropoff", py, px, color=object_color)

    return image


BASE_COLORS = [
    "red", "green", "blue", "yellow", "orange", "pink", "purple", "brown",
    "gray", "beige", "turquoise", "cyan", "magenta", "lime", "navy",
    "maroon", "teal", "olive", "coral", "lavender", "violet", "gold", "silver"
]

ADDITIONAL_COLORS = [
    colorname for (colorname, colorcode) in ImageColor.colormap.items()
]

ALL_COLORS = BASE_COLORS + ADDITIONAL_COLORS

def get_color_for_label(label: str) -> tuple:
    """
    Attempts to get the actual BGR color for the label (e.g., 'red' -> red box).
    Falls back to a deterministic, non-flickering random color if the name is unknown.
    """
    try:
        # getrgb returns (R, G, B)
        r, g, b = ImageColor.getrgb(label.lower().strip())
        return (b, g, r) # OpenCV needs BGR
    except ValueError:
        # Fallback for hallucinated color names (deterministic, no flickering)
        random.seed(label) 
        b = random.randint(50, 255)
        g = random.randint(50, 255)
        r = random.randint(50, 255)
        return (b, g, r)



# --- ROS2 NODE ---

class GeminiVisionNode(Node):
    def __init__(self):
        super().__init__('gemini_vision_node')



        # Declare the parameter with an empty string as the default
        self.declare_parameter('gemini_model', '')
        passed_model = self.get_parameter('gemini_model').value

        # Fallback logic: If the launch file passed an empty string, use the global constant.
        # Otherwise, use the model passed via the command line.
        if passed_model == '':
            self.gemini_model = GEMINI_MODEL
        else:
            self.gemini_model = passed_model

        self.api_key = os.environ.get('GEMINI_API_KEY')
        if not self.api_key:
            self.get_logger().fatal("🔴 GEMINI_API_KEY not set!")
            raise RuntimeError("Missing API key")
        self.client = genai.Client(api_key=self.api_key)
        self.get_logger().info(f"🟢 Model: {self.gemini_model} - ready.")

        
        # --- ROS Parameters ---

        # Camera Topics
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('color_image_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_image_topic', '/camera/aligned_depth_to_color/image_raw')

        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.camera_color_image_topic = self.get_parameter('color_image_topic').value
        self.camera_depth_image_topic = self.get_parameter('depth_image_topic').value

        # 3D & TF2 Parameters
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('robot_base_frame', 'base_link')
        self.declare_parameter('object_center_offset', 0.0) # Offset from front face to center of object in meters (for 3D grasping)
        self.declare_parameter('max_depth_m', 10.0) # Ignore detections beyond this depth (in meters)

        self.camera_frame = self.get_parameter('camera_frame').value
        self.robot_base_frame = self.get_parameter('robot_base_frame').value
        self.object_center_offset = self.get_parameter('object_center_offset').value
        self.max_depth_m = self.get_parameter('max_depth_m').value

        # --- TF2 Setup ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- Image & Detection State ---
        self.bridge = CvBridge()
        self.camera_model = PinholeCameraModel()
        self.camera_info_ready = False
        self.latest_color_msg: Optional[Image] = None
        self.latest_depth_msg: Optional[Image] = None
        self.last_detections: List[ObjectDetection] = []
        self._detection_lock = Lock() # Thread safety lock

        # --- ROS Interface (Subscriptions & Publishers) ---
        self.camera_info_sub = self.create_subscription(CameraInfo, self.camera_info_topic, self._camera_info_callback, 5)
        self.color_image_sub = self.create_subscription(Image, self.camera_color_image_topic, self._color_image_callback, 5)
        self.depth_image_sub = self.create_subscription(Image, self.camera_depth_image_topic, self._depth_image_callback, 5)

        self.annotated_image_pub = self.create_publisher(Image, '/annotated_image', 5)
        self.vis_timer = self.create_timer(0.1, self._annotation_timer_callback) # 10 Hz for visualization

        # --- Service Server (Multithreaded) ---
        self.srv_cb_group = MutuallyExclusiveCallbackGroup()
        self.create_service(DetectObjects, '/detect_objects', self.execute_detection_pipeline, callback_group=self.srv_cb_group)
        
        # --- Startup Status ---
        self.get_logger().info(
            "Status:\n" +
            "="*60 + "\n" +
            "🟢 GeminiVisionNode (Service Node) ready. 🟢\n" +
            f"📷 Subscribed to camera topics: {self.camera_info_topic}, {self.camera_color_image_topic}, {self.camera_depth_image_topic}\n" +
            "👂 Waiting for service call...\n" +
            "\nTo test functionality manually, use:\n" +
            "🅰️  Default Mode - 'Detects all bricks on the table':\n" +
            "ros2 service call /detect_objects object_interfaces/srv/DetectObjects\n" +
            "🅱️  User Prompt Mode - add your instructions to the data field:\n" +
            "ros2 service call /detect_objects object_interfaces/srv/DetectObjects \"{user_prompt: 'Pick the red bricks'}\"\n" +
            "="*60
        )


    # --- MAIN PIPELINE ---

    def execute_detection_pipeline(self, request, response):
        """Service-Handler: Triggers Gemini detection and returns results."""
        self.get_logger().info("Detection triggered.")

        if self.latest_color_msg is None or self.latest_depth_msg is None:
            response.success = False
            response.message = "🔴 No color or depth image available yet."
            self.get_logger().info(response.message)
            return response

        if not self.camera_info_ready:
            response.success = False
            response.message = "🔴 Camera info not ready yet."
            self.get_logger().info(response.message)
            return response

        try:
            # 1. Convert Color Image for Gemini
            image_bytes = self._prepare_image_for_gemini_cv(self.latest_color_msg)

            # 2. Convert Depth Image for 3D Projection (strictly to mm as float)
            if self.latest_depth_msg.encoding == '32FC1':
                depth_meters = self.bridge.imgmsg_to_cv2(self.latest_depth_msg, desired_encoding="32FC1")
                cv_depth = depth_meters * 1000.0
            else:
                cv_depth = self.bridge.imgmsg_to_cv2(self.latest_depth_msg, desired_encoding="16UC1")

            # 3. Call Gemini API
            detections = self._call_gemini(image_bytes, request.user_prompt)

            if detections is not None:
                # 4. Process 3D Coordinates & build ROS Messages
                img_h = self.latest_color_msg.height
                img_w = self.latest_color_msg.width
                target_object_msgs = self._process_3d_detections(detections, cv_depth, img_w, img_h)

                with self._detection_lock:
                    self.last_detections = detections
                
                response.success = True
                response.message = f"Detected {len(target_object_msgs)} valid objects."
                response.objects = target_object_msgs

                # --- Logging the 3D Coordinates ---
                if len(target_object_msgs) > 0:
                    log_msg = f"\n{'='*60}\n"
                    log_msg += f"🏁 Detected {len(target_object_msgs)} valid objects:\n"
                    object_number = 0
                    for object in target_object_msgs:
                        object_number += 1
                        x = object.position.point.x
                        y = object.position.point.y
                        z = object.position.point.z
                        color = object.color.data
                        distance = object.camera_distance_mm
                        
                        # String for drop-off coordinates (only if user_dropoff is True)
                        dropoff_str = ""
                        if object.has_user_dropoff:
                            dx = object.user_dropoff_position.x
                            dy = object.user_dropoff_position.y
                            dropoff_str = f" | User-Drop-off: [X: {dx:.3f}, Y: {dy:.3f}]"
                        else:
                            dropoff_str = " | Default Drop-off (see workcell_bringup *_workspace_parameters.yaml)"

                        # Log format: "- Red: [X: 0.123, Y: 0.456, Z: 0.789] | Distance: 500 mm | Drop-off: [X: 0.200, Y: 0.300]"
                        log_msg += f"{object_number} - {color.capitalize()}: [X: {x:.3f}, Y: {y:.3f}, Z: {z:.3f}] | Distance: {distance:.0f} mm{dropoff_str}\n"
                        
                    log_msg += f"{'='*60}"
                    self.get_logger().info(log_msg)

            else:
                with self._detection_lock:
                    self.last_detections = []
                response.success = False
                response.message = "🔴 Gemini API call failed"

        except Exception as e:
            self.get_logger().error(f"🔴 Detection error: {e}")
            self.last_detections = []
            response.success = False
            response.message = str(e)

        self.get_logger().info(f"Detection process finished. Success: {response.success}. Message: {response.message}")

        self.get_logger().info(
            "Status:\n" + "="*60 + "\n" +
            "🟢 GeminiVisionNode (Service Node) ready. 🟢\n" +
            "👂 Waiting for service call...\n" +
            "="*60
        )

        return response


    # --- CALLBACKS ---

    def _camera_info_callback(self, msg: CameraInfo):
        """Initializes the PinholeCameraModel with the camera's intrinsic matrix."""
        if not self.camera_info_ready:
            self.camera_info_ready = True
            self.get_logger().info("Camera intrinsics received. 3D projection ready.")

        self.camera_model.fromCameraInfo(msg)

    def _color_image_callback(self, msg: Image):
        """Saves the latest color image for processing."""
        self.latest_color_msg = msg

    def _depth_image_callback(self, msg: Image):
        """Saves the latest depth image for 3D projection."""
        self.latest_depth_msg = msg

    def _annotation_timer_callback(self):
        """Annotates the latest image and publishes it for visualization."""
        if self.latest_color_msg is None:
            return
        try:
            annotated = self._build_annotated_image()
            msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            msg.header = self.latest_color_msg.header
            self.annotated_image_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"🔴 Annotation error: {e}", throttle_duration_sec=5.0)


    # --- HELPER FUNCTIONS ---

    def _prepare_image_for_gemini(self, ros_msg: Image) -> bytes:
        """ROS Image → JPEG Bytes"""
        # 1. Convert ROS message to OpenCV (RGB)
        image_rgb = self.bridge.imgmsg_to_cv2(ros_msg, 'rgb8')

        # 2. Convert NumPy array to PIL
        pil = PILImage.fromarray(image_rgb)


        # 3. Convert PIL Image to Bytes (JPEG) with high quality
        img_byte_arr = io.BytesIO()
        pil.save(img_byte_arr, format='JPEG', quality=95)

        # 4. Extract byte data into a distinct variable before returning
        image_bytes = img_byte_arr.getvalue()

        return image_bytes

    def _prepare_image_for_gemini_cv(self, ros_msg: Image) -> bytes:
        """ROS Image → OpenCV → JPEG Bytes (High Performance)."""
        
        # 1. Convert ROS message to OpenCV (BGR is OpenCV's default format)
        image_bgr = self.bridge.imgmsg_to_cv2(ros_msg, 'bgr8')

        # 2. Encode directly to JPEG in memory (Quality: 95)
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 95]
        success, encoded_image = cv2.imencode('.jpg', image_bgr, encode_param)

        if not success:
            self.get_logger().error("🔴 Failed to encode image to JPEG!")
            return b""

        # 3. Convert the numpy array to raw bytes for the Gemini API
        return encoded_image.tobytes()


    def _build_annotated_image(self) -> np.ndarray:
        """Builds an annotated image by converting the ROS message and calling the drawing helper."""
        # Convert ROS image to OpenCV BGR
        image_bgr = self.bridge.imgmsg_to_cv2(self.latest_color_msg, "bgr8").copy()
        # Get image dimensions needed for normalization
        img_h, img_w = image_bgr.shape[:2]

        # Safely copy the list while the lock is acquired
        with self._detection_lock:
            current_detections = list(self.last_detections)

        # Pass the image, the detections, and the dimensions to the pure helper function
        image_annotated = draw_all_annotations(image_bgr, current_detections, img_w, img_h)

        return image_annotated


    def _call_gemini(self, img_bytes: bytes, user_prompt: Optional[str] = None) -> Optional[List[ObjectDetection]]:
        """Calls the Gemini API with the image and prompt, returns list of detections."""
        start = time.time()
        prompt = build_prompt(user_prompt)
        
        current_temperature = 1.0 # Default temperature, recommended to keep it at 1.0, can be adjusted based on user needs (e.g., lower for more deterministic results, higher for more creative interpretations)
        
        # --- Thinking Config Selection Logic ---

        # 1. Specific Config for Gemini Robotics-ER 1.5 (uses thinking_budget instead of thinking_level)
        if "gemini-robotics-er-1.5" in self.gemini_model:
            current_thinking_config = types.ThinkingConfig(
                thinking_budget=GEMINI_THINKING_BUDGET,
                include_thoughts=True
            )
            self.get_logger().info(f"Using thinking_budget={GEMINI_THINKING_BUDGET} for model {self.gemini_model}")
            
        # 2. Specific Config for Gemini 3 Family (supporting thinking_level)
        elif "gemini-3" in self.gemini_model: 
            if not user_prompt:
                current_thinking_level = "minimal"
                self.get_logger().info("🅰️  Default Mode:")
            else:
                self.get_logger().info("🅱️  User Prompt Mode:")
                if self.gemini_model == "gemini-3-flash-preview":
                    current_thinking_level = GEMINI_THINKING_LEVEL_3FLASH
                elif self.gemini_model == "gemini-3.1-flash-lite-preview":
                    current_thinking_level = GEMINI_THINKING_LEVEL_31FLASHLITE
                else:
                    # Fallback für zukünftige Gemini 3 Modelle
                    current_thinking_level = GEMINI_THINKING_LEVEL_DEFAULT 
                    
            current_thinking_config = types.ThinkingConfig(
                thinking_level=current_thinking_level,
                include_thoughts=True
            )
            self.get_logger().info(f"Using thinking_level='{current_thinking_level}' for model {self.gemini_model}")
            
        # 3. Universal fallback for unknown or older models (e.g., set via launch file)
        else:
            if not user_prompt:
                self.get_logger().info("🅰️  Default Mode (Fallback Model):")
            else:
                self.get_logger().info("🅱️  User Prompt Mode (Fallback Model):")
                
            # For unknown models, we use a safe default that includes thoughts but does not rely on specific thinking_level or thinking_budget parameters.
            current_thinking_config = types.ThinkingConfig(
                include_thoughts=True
            )
            self.get_logger().info(f"Using default thinking config (only include_thoughts=True) for model {self.gemini_model}")

        # --- Gemini API Call ---

        self.get_logger().info(f"Prompt: {prompt}")

        try:
            # Create the image part separately to ensure it's distinct in memory and not optimized away
            image_part = types.Part.from_bytes(
                data=img_bytes,
                mime_type='image/jpeg'
            )
            
            resp = self.client.models.generate_content(
                model=self.gemini_model,
                contents=[prompt, image_part],
                config=types.GenerateContentConfig(
                    system_instruction=GEMINI_SYSTEM_PROMPT,
                    response_mime_type="application/json",
                    response_json_schema=DetectionResult.model_json_schema(),
                    temperature=current_temperature,
                    thinking_config=current_thinking_config,
                ),
            )

            self._log_gemini_response(resp)
            self.get_logger().info(f"⏱️  Processing time: {time.time() - start:.2f}s")

            result = DetectionResult.model_validate_json(resp.text)
            return result.objects

        except Exception as e:
            self.get_logger().error(f"🔴 Gemini error: {e}")
            return None


    def _log_gemini_response(self, resp):
        """Logs Thoughts and Answer separately."""
        for part in resp.candidates[0].content.parts:
            if not part.text:
                continue
            tag = "Thought" if part.thought else "Answer"
            self.get_logger().info(f"\n{'='*60}\n{tag}:\n{part.text}")


    def _process_3d_detections(self, detections: List[ObjectDetection], cv_depth: np.ndarray, img_w: int, img_h: int) -> List[DetectedObject]:
        """Calculates 3D coordinates, updates Pydantic objects, and builds ROS messages."""
        target_object_msgs = [] # 
        object_counter = {}

        # 1. Look up the TF transform once for the entire batch
        try:
            tf_cam_to_base = self.tf_buffer.lookup_transform(
                self.robot_base_frame, self.camera_frame,
                rclpy.time.Time(), rclpy.duration.Duration(seconds=1.0)
            )
        except Exception as e:
            self.get_logger().error(f"TF lookup failed: {e}")
            return []

        for object in detections:
            color = object.label.lower()
            ymin, xmin, ymax, xmax = normalized_to_pixel(object.box_2d, img_w, img_h)
            
            # --- Aspect Ratio & Yaw ---
            w = xmax - xmin
            h = ymax - ymin
            aspect_ratio = w / h if h > 0 else 0.0
            object.yaw_degrees = 0.0 if 0 < aspect_ratio < 1.41 else 30.0

            # --- Robust Depth for Object ---
            depth_mm = get_robust_depth(object.box_2d, cv_depth, img_w, img_h)
            
            if depth_mm is not None:
                z = (depth_mm / 1000.0) + self.object_center_offset
                
                if 0.03 < z <= self.max_depth_m:
                    # 2D Center to 3D Ray projection
                    cx = np.clip((xmin + xmax) // 2, 0, img_w - 1)
                    cy = np.clip((ymin + ymax) // 2, 0, img_h - 1)
                    ray = self.camera_model.projectPixelTo3dRay((cx, cy))
                    scale = z / ray[2]
                    
                    # Create Point in Camera Frame
                    pt_cam = PointStamped()
                    pt_cam.header.frame_id = self.camera_frame
                    pt_cam.point.x = ray[0] * scale
                    pt_cam.point.y = ray[1] * scale
                    pt_cam.point.z = z

                    # Transform to Robot Base Frame
                    pt_base = do_transform_point(pt_cam, tf_cam_to_base)
                    
                    # Store in Pydantic for drawing
                    object.position_3d = [pt_base.point.x, pt_base.point.y, pt_base.point.z]
                    
                    # --- Debug ---
                    self.get_logger().info(
                        f"DEBUG {color}: pixel=({cx},{cy}), "
                        f"ray=({ray[0]:.4f},{ray[1]:.4f},{ray[2]:.4f}), "
                        f"scale={scale:.4f}, "
                        f"cam=({ray[0]*scale:.4f},{ray[1]*scale:.4f},{z:.4f}), "
                        f"base=({pt_base.point.x:.4f},{pt_base.point.y:.4f},{pt_base.point.z:.4f})"
                    )

                    # --- Build ROS Message ---
                    msg = DetectedObject()
                    msg.color.data = color
                    msg.position = pt_base
                    msg.camera_distance_mm = z * 1000.0
                    msg.yaw_degrees = object.yaw_degrees
                    msg.bounding_box_px = [ymin, xmin, ymax, xmax]
                    msg.has_user_dropoff = False

                    # --- Robust Depth for Drop-off ---
                    # --- 1. Visual Drop-off (2D Point provided by Gemini) ---
                    if object.dropoff_point_2d:
                        drop_depth_mm = get_robust_depth(object.dropoff_point_2d, cv_depth, img_w, img_h)

                        # Fallback to object depth if dropoff pixel has no depth
                        if drop_depth_mm is None:
                            self.get_logger().warn(
                                f"No depth at dropoff for '{color}'. Using object depth as fallback."
                            )
                            drop_depth_mm = depth_mm
    

                        if drop_depth_mm is not None:
                            drop_z = drop_depth_mm / 1000.0
                            dy, dx = normalized_to_pixel(object.dropoff_point_2d, img_w, img_h)
                            drop_ray = self.camera_model.projectPixelTo3dRay((dx, dy))
                            scale = drop_z / drop_ray[2]
                            
                            drop_cam = PointStamped()
                            drop_cam.header.frame_id = self.camera_frame
                            drop_cam.point.x = drop_ray[0] * scale
                            drop_cam.point.y = drop_ray[1] * scale
                            drop_cam.point.z = drop_z
                            
                            drop_base = do_transform_point(drop_cam, tf_cam_to_base)
                            object.dropoff_3d = [drop_base.point.x, drop_base.point.y, drop_base.point.z]
                            
                            msg.has_user_dropoff = True
                            msg.user_dropoff_position = drop_base.point

                    # --- 2. Direct Metric Drop-off (Explicit X/Y provided by Gemini) ---
                    elif object.dropoff_coords_m:
                        msg.has_user_dropoff = True
                        msg.user_dropoff_position.x = float(object.dropoff_coords_m[0])
                        msg.user_dropoff_position.y = float(object.dropoff_coords_m[1])
                        # Use the Z coordinate (height) of the detected object as the table surface height
                        msg.user_dropoff_position.z = pt_base.point.z
                        
                        # Store in Pydantic for internal handling
                        object.dropoff_3d = [msg.user_dropoff_position.x, msg.user_dropoff_position.y, msg.user_dropoff_position.z]

                    target_object_msgs.append(msg)

                    # --- TF Broadcasting ---
                    count = object_counter.get(color, 0)
                    object_counter[color] = count + 1
                    child_frame = f"object_{color}_{count}"
                    
                    t = TransformStamped()
                    t.header.stamp = self.get_clock().now().to_msg()
                    t.header.frame_id = self.robot_base_frame
                    t.child_frame_id = child_frame
                    t.transform.translation.x = pt_base.point.x
                    t.transform.translation.y = pt_base.point.y
                    t.transform.translation.z = pt_base.point.z
                    
                    yaw_rad = math.radians(object.yaw_degrees)
                    t.transform.rotation.z = math.sin(yaw_rad / 2.0)
                    t.transform.rotation.w = math.cos(yaw_rad / 2.0)
                    self.tf_broadcaster.sendTransform(t)

        return target_object_msgs



# --- Main ---

def main(args=None):
    rclpy.init(args=args)
    node = GeminiVisionNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): 
            rclpy.shutdown()


if __name__ == '__main__':
    main()