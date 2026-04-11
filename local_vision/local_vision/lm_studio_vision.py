#!/usr/bin/env python3

''' 
A vision system for a robotic pick-and-place task using the LM Studio API.
Input: RGB image. Output: Detected objects with bounding boxes, colors, and 
potential drop-off locations based on user instructions (optional).
The node subscribes to camera topics, processes images with a local LLM hosted
in LM Studio, and returns 3D coordinates for detected objects.
'''

import cv2
import numpy as np
from cv_bridge import CvBridge
import textwrap
import time
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
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from PIL import ImageColor
from pydantic import BaseModel, Field
from typing import List, Optional
import openai
import base64


# ---------------------------------------------------------------------------
# LM STUDIO CONFIGURATION
# ---------------------------------------------------------------------------

LM_STUDIO_MODELS = [
    "google/gemma-4-e2b",        # [0]  5.95 GB, Q8
    "google/gemma-4-e4b",        # [1]  6.33 GB, Q8
    "google/gemma-4-26b-a4b",    # [2] 17.99 GB, Q4
    "qwen/qwen3.5-9b",           # [3] 10.45 GB, Q8
]
LM_STUDIO_MODEL = LM_STUDIO_MODELS[0]  # ← Default hier wechseln

LM_STUDIO_API_KEY   = "lm-studio"   # Wird von LM Studio ignoriert, muss aber gesetzt sein
LM_STUDIO_TEMPERATURE = 1.0          # Empfohlen: 1.0 (analog zu Gemini)

DEFAULT_PROMPT = textwrap.dedent("""\
    Detect all bricks on the table
""")

SYSTEM_PROMPT = textwrap.dedent("""\
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


# ---------------------------------------------------------------------------
# PYDANTIC MODELS
# ---------------------------------------------------------------------------

class ObjectDetection(BaseModel):
    object_name: str = Field(
        description="Name/type of the object (e.g., 'brick', 'pen')")
    label: str = Field(
        description="Dominant color of the object (e.g., 'red', 'blue')")
    box_2d: List[int] = Field(
        description="[ymin, xmin, ymax, xmax] normalized 0-1000")
    user_dropoff: bool = Field(
        default=False,
        description="True ONLY if the user explicitly specified a custom "
                    "drop-off location or target for this object.")
    dropoff_point_2d: Optional[List[int]] = Field(
        default=None,
        description="[y, x] normalized 0-1000. Use ONLY if the custom "
                    "drop-off is a visual feature visible in the image.")
    dropoff_coords_m: Optional[List[float]] = Field(
        default=None,
        description="[x, y] in meters. Use ONLY if the user explicitly "
                    "provided numerical metric coordinates in the prompt.")
    # Internal fields – excluded from JSON schema sent to the LLM
    position_3d:   Optional[List[float]] = Field(default=None, exclude=True)
    dropoff_3d:    Optional[List[float]] = Field(default=None, exclude=True)
    yaw_degrees:   float                 = Field(default=0.0,  exclude=True)


class DetectionResult(BaseModel):
    objects: List[ObjectDetection]


# ---------------------------------------------------------------------------
# HELPER FUNCTIONS
# ---------------------------------------------------------------------------

def build_prompt(user_prompt: Optional[str] = None) -> str:
    """Returns the user-facing prompt string."""
    if user_prompt:
        return textwrap.dedent(user_prompt)
    return DEFAULT_PROMPT


def normalized_to_pixel(coords: List[int], img_w: int, img_h: int) -> tuple:
    """Converts normalized 0-1000 coordinates to absolute pixel coordinates."""
    if len(coords) == 4:
        ymin_n, xmin_n, ymax_n, xmax_n = coords
        y1 = int(ymin_n / 1000.0 * img_h)
        x1 = int(xmin_n / 1000.0 * img_w)
        y2 = int(ymax_n / 1000.0 * img_h)
        x2 = int(xmax_n / 1000.0 * img_w)
        return (min(y1, y2), min(x1, x2), max(y1, y2), max(x1, x2))
    elif len(coords) == 2:
        y_n, x_n = coords
        return (int(y_n / 1000.0 * img_h), int(x_n / 1000.0 * img_w))
    else:
        raise ValueError(f"Expected 2 or 4 coordinates, got {len(coords)}")


def get_robust_depth(coords_2d: List[int], cv_depth: np.ndarray,
                     img_w: int, img_h: int) -> Optional[float]:
    """Extracts median depth from an adaptive region around the bbox centre."""
    if len(coords_2d) == 4:
        ymin, xmin, ymax, xmax = normalized_to_pixel(coords_2d, img_w, img_h)
        cx = (xmin + xmax) // 2
        cy = (ymin + ymax) // 2
        box_w = xmax - xmin
        box_h = ymax - ymin
        half_w = max(3, min(20, int(box_w * 0.15)))
        half_h = max(3, min(20, int(box_h * 0.15)))
    elif len(coords_2d) == 2:
        cy, cx = normalized_to_pixel(coords_2d, img_w, img_h)
        half_w = half_h = 5
    else:
        return None

    cx = int(np.clip(cx, 0, img_w - 1))
    cy = int(np.clip(cy, 0, img_h - 1))

    region = cv_depth[
        max(0, cy - half_h):min(img_h, cy + half_h + 1),
        max(0, cx - half_w):min(img_w, cx + half_w + 1)
    ]
    valid = region[(region > 0) & np.isfinite(region)]
    return float(np.median(valid)) if valid.size > 0 else None


def draw_bbox(image, label, ymin, xmin, ymax, xmax,
              pt_3d=None, color=(255, 140, 72)):
    """Draws a bounding box with label and optional 3D coordinates."""
    cv2.rectangle(image, (xmin, ymin), (xmax, ymax), (0, 0, 0), 3)
    cv2.rectangle(image, (xmin, ymin), (xmax, ymax), color, 1)
    cx, cy = (xmin + xmax) // 2, (ymin + ymax) // 2
    cv2.circle(image, (cx, cy), 4, (0, 0, 0), -1)
    cv2.circle(image, (cx, cy), 2, color, -1)

    outline, fg = (255, 255, 255), (0, 0, 0)
    pos1 = (xmin, max(35, ymin - 20))
    cv2.putText(image, label, pos1, cv2.FONT_HERSHEY_SIMPLEX,
                0.45, outline, 3, cv2.LINE_AA)
    cv2.putText(image, label, pos1, cv2.FONT_HERSHEY_SIMPLEX,
                0.45, fg,     1, cv2.LINE_AA)

    pos2 = (xmin, max(20, ymin - 5))
    if pt_3d:
        coord_text = f"X:{pt_3d[0]:.2f} Y:{pt_3d[1]:.2f}"
        cv2.putText(image, coord_text, pos2, cv2.FONT_HERSHEY_SIMPLEX,
                    0.35, outline, 3, cv2.LINE_AA)
        cv2.putText(image, coord_text, pos2, cv2.FONT_HERSHEY_SIMPLEX,
                    0.35, fg,     1, cv2.LINE_AA)
    else:
        cv2.putText(image, "(no depth)", pos2, cv2.FONT_HERSHEY_SIMPLEX,
                    0.35, (0, 0, 255), 2, cv2.LINE_AA)


def draw_point(image, label, y, x, color=(72, 255, 140)):
    """Draws a cross-marker with label at the given pixel position."""
    cv2.drawMarker(image, (x, y), (0, 0, 0),
                   markerType=cv2.MARKER_CROSS, markerSize=20, thickness=3)
    cv2.drawMarker(image, (x, y), color,
                   markerType=cv2.MARKER_CROSS, markerSize=20, thickness=1)
    outline, fg = (255, 255, 255), (0, 0, 0)
    pos = (x + 10, y - 10)
    cv2.putText(image, label, pos, cv2.FONT_HERSHEY_SIMPLEX,
                0.45, outline, 3, cv2.LINE_AA)
    cv2.putText(image, label, pos, cv2.FONT_HERSHEY_SIMPLEX,
                0.45, fg,     1, cv2.LINE_AA)


def draw_all_annotations(image: np.ndarray, detections: List[ObjectDetection],
                          img_w: int, img_h: int) -> np.ndarray:
    """Draws all bounding boxes and drop-off points on the image."""
    for obj in detections:
        obj_color    = get_color_for_label(obj.label)
        display_text = f"{obj.label} {obj.object_name}"

        ymin, xmin, ymax, xmax = normalized_to_pixel(obj.box_2d, img_w, img_h)
        draw_bbox(image, display_text, ymin, xmin, ymax, xmax,
                  pt_3d=obj.position_3d, color=obj_color)

        if obj.dropoff_point_2d is not None:
            py, px = normalized_to_pixel(obj.dropoff_point_2d, img_w, img_h)
            draw_point(image, f"{display_text} dropoff", py, px,
                       color=obj_color)
    return image


def get_color_for_label(label: str) -> tuple:
    """Returns a BGR colour for a colour-name label, with deterministic fallback."""
    try:
        r, g, b = ImageColor.getrgb(label.lower().strip())
        return (b, g, r)
    except ValueError:
        random.seed(label)
        return (random.randint(50, 255),
                random.randint(50, 255),
                random.randint(50, 255))


# ---------------------------------------------------------------------------
# ROS 2 NODE
# ---------------------------------------------------------------------------

class LmStudioVisionNode(Node):

    def __init__(self):
        super().__init__('lm_studio_vision_node')

        # --- Model selection (launch argument overrides script default) ---
        # dynamic_typing=True erlaubt es dem Terminal, sowohl Zahlen als auch Texte zu schicken
        self.declare_parameter('lm_studio_model', '', ParameterDescriptor(dynamic_typing=True))
        
        # Egal ob Zahl (1) oder String ("qwen"), wir machen daraus sofort einen Python-String
        passed_model = str(self.get_parameter('lm_studio_model').value)

        if passed_model == '':
            # Kein Argument → Script-Default
            self.lm_studio_model = LM_STUDIO_MODEL

        elif passed_model.isdigit():
            # Zahl übergeben → als Index in die Liste
            idx = int(passed_model)
            if 0 <= idx < len(LM_STUDIO_MODELS):
                self.lm_studio_model = LM_STUDIO_MODELS[idx]
            else:
                self.get_logger().error(
                    f"🔴 Model index {idx} out of range "
                    f"(0–{len(LM_STUDIO_MODELS)-1}). Using default."
                )
                self.lm_studio_model = LM_STUDIO_MODEL

        else:
            # Voller Name übergeben → direkt verwenden
            self.lm_studio_model = passed_model


        # --- LM Studio host / port from YAML ---
        self.declare_parameter('lm_studio_host', '127.0.0.1')
        self.declare_parameter('lm_studio_port', 1234)
        host = self.get_parameter('lm_studio_host').value
        port = self.get_parameter('lm_studio_port').value
        base_url = f"http://{host}:{port}/v1"

        self.lm_client = openai.OpenAI(base_url=base_url,
                                       api_key=LM_STUDIO_API_KEY)
        try:
            models = self.lm_client.models.list()
            loaded = [m.id for m in models.data]
            self.get_logger().info(f"📋 Loaded models in LM Studio: {loaded}")
        except Exception as e:
            self.get_logger().warn(f"Could not query LM Studio models: {e}")
        self.get_logger().info(
            f"🟢 LM Studio: {base_url} | Model: {self.lm_studio_model}"
        )

        # --- Camera topics ---
        self.declare_parameter('camera_info_topic',
                               '/camera/color/camera_info')
        self.declare_parameter('color_image_topic',
                               '/camera/color/image_raw')
        self.declare_parameter('depth_image_topic',
                               '/camera/aligned_depth_to_color/image_raw')
        self.camera_info_topic        = self.get_parameter('camera_info_topic').value
        self.camera_color_image_topic = self.get_parameter('color_image_topic').value
        self.camera_depth_image_topic = self.get_parameter('depth_image_topic').value

        # --- 3D / TF2 parameters ---
        self.declare_parameter('camera_frame',         'camera_color_optical_frame')
        self.declare_parameter('robot_base_frame',     'base_link')
        self.declare_parameter('object_center_offset', 0.0)
        self.declare_parameter('max_depth_m',          10.0)
        self.camera_frame          = self.get_parameter('camera_frame').value
        self.robot_base_frame      = self.get_parameter('robot_base_frame').value
        self.object_center_offset  = self.get_parameter('object_center_offset').value
        self.max_depth_m           = self.get_parameter('max_depth_m').value

        # --- TF2 ---
        self.tf_buffer      = Buffer()
        self.tf_listener    = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- State ---
        self.bridge             = CvBridge()
        self.camera_model       = PinholeCameraModel()
        self.camera_info_ready  = False
        self.latest_color_msg:  Optional[Image] = None
        self.latest_depth_msg:  Optional[Image] = None
        self.last_detections:   List[ObjectDetection] = []
        self._detection_lock    = Lock()

        # --- Subscriptions & publishers ---
        self.create_subscription(CameraInfo, self.camera_info_topic,
                                 self._camera_info_callback, 5)
        self.create_subscription(Image, self.camera_color_image_topic,
                                 self._color_image_callback, 5)
        self.create_subscription(Image, self.camera_depth_image_topic,
                                 self._depth_image_callback, 5)

        self.annotated_image_pub = self.create_publisher(
            Image, '/annotated_image', 5)
        self.create_timer(0.1, self._annotation_timer_callback)

        # --- Service (multi-threaded) ---
        self.srv_cb_group = MutuallyExclusiveCallbackGroup()
        self.create_service(DetectObjects, '/detect_objects',
                            self.execute_detection_pipeline,
                            callback_group=self.srv_cb_group)

        # --- Startup banner ---
        self.get_logger().info(
            "Status:\n" + "=" * 60 + "\n"
            "🟢 LM Studio Vision Node (Service Node) ready. 🟢\n"
            f"📷 Camera topics: {self.camera_info_topic}, "
            f"{self.camera_color_image_topic}, "
            f"{self.camera_depth_image_topic}\n"
            "👂 Waiting for service call...\n\n"
            "To test manually:\n"
            "🅰️  Default Mode:\n"
            "    ros2 service call /detect_objects "
            "object_interfaces/srv/DetectObjects\n"
            "🅱️  User Prompt Mode:\n"
            "    ros2 service call /detect_objects "
            "object_interfaces/srv/DetectObjects "
            "\"{user_prompt: 'Pick the red bricks'}\"\n"
            + "=" * 60
        )

    # -----------------------------------------------------------------------
    # MAIN PIPELINE
    # -----------------------------------------------------------------------

    def execute_detection_pipeline(self, request, response):
        """Service handler: triggers LM Studio detection and returns results."""
        self.get_logger().info("Detection triggered.")

        if self.latest_color_msg is None or self.latest_depth_msg is None:
            response.success = False
            response.message = "🔴 No color or depth image available yet."
            self.get_logger().warn(response.message)
            return response

        if not self.camera_info_ready:
            response.success = False
            response.message = "🔴 Camera info not ready yet."
            self.get_logger().warn(response.message)
            return response

        try:
            # 1. Prepare colour image (JPEG bytes)
            image_bytes = self._prepare_image_bytes(self.latest_color_msg)

            # 2. Prepare depth image (mm, float32)
            if self.latest_depth_msg.encoding == '32FC1':
                cv_depth = (self.bridge.imgmsg_to_cv2(
                    self.latest_depth_msg, desired_encoding="32FC1") * 1000.0)
            else:
                cv_depth = self.bridge.imgmsg_to_cv2(
                    self.latest_depth_msg,
                    desired_encoding="16UC1").astype(np.float32)

            # 3. Call LM Studio
            detections = self._call_lm_studio(image_bytes, request.user_prompt)

            if detections is not None:
                img_h = self.latest_color_msg.height
                img_w = self.latest_color_msg.width

                # 4. 3D projection + ROS message building
                target_object_msgs = self._process_3d_detections(
                    detections, cv_depth, img_w, img_h)

                with self._detection_lock:
                    self.last_detections = detections

                response.success = True
                response.message = (
                    f"Detected {len(target_object_msgs)} valid objects.")
                response.objects = target_object_msgs

                # 5. Logging
                if target_object_msgs:
                    log_msg = f"\n{'=' * 60}\n"
                    log_msg += (f"🏁 Detected {len(target_object_msgs)} "
                                f"valid objects:\n")
                    for i, obj in enumerate(target_object_msgs, start=1):
                        x     = obj.position.point.x
                        y     = obj.position.point.y
                        z     = obj.position.point.z
                        color = obj.color.data
                        dist  = obj.camera_distance_mm
                        if obj.has_user_dropoff:
                            dx  = obj.user_dropoff_position.x
                            dy  = obj.user_dropoff_position.y
                            dro = f" | User-Drop-off: [X:{dx:.3f} Y:{dy:.3f}]"
                        else:
                            dro = (" | Default Drop-off "
                                   "(see workcell_bringup "
                                   "*_workspace_parameters.yaml)")
                        log_msg += (
                            f"{i} - {color.capitalize()}: "
                            f"[X:{x:.3f} Y:{y:.3f} Z:{z:.3f}] "
                            f"| Distance: {dist:.0f} mm{dro}\n"
                        )
                    log_msg += "=" * 60
                    self.get_logger().info(log_msg)

            else:
                with self._detection_lock:
                    self.last_detections = []
                response.success = False
                response.message = "🔴 LM Studio API call failed."

        except Exception as e:
            self.get_logger().error(f"🔴 Detection error: {e}")
            with self._detection_lock:
                self.last_detections = []
            response.success = False
            response.message = str(e)

        self.get_logger().info(
            f"Detection finished. Success: {response.success}. "
            f"Message: {response.message}"
        )
        self.get_logger().info(
            "Status:\n" + "=" * 60 + "\n"
            "🟢 LM Studio Vision Node (Service Node) ready. 🟢\n"
            "👂 Waiting for service call...\n"
            + "=" * 60
        )
        return response

    # -----------------------------------------------------------------------
    # CALLBACKS
    # -----------------------------------------------------------------------

    def _camera_info_callback(self, msg: CameraInfo):
        if not self.camera_info_ready:
            self.camera_info_ready = True
            self.get_logger().info(
                "Camera intrinsics received. 3D projection ready.")
        self.camera_model.fromCameraInfo(msg)

    def _color_image_callback(self, msg: Image):
        self.latest_color_msg = msg

    def _depth_image_callback(self, msg: Image):
        self.latest_depth_msg = msg

    def _annotation_timer_callback(self):
        if self.latest_color_msg is None:
            return
        try:
            annotated = self._build_annotated_image()
            out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            out_msg.header = self.latest_color_msg.header
            self.annotated_image_pub.publish(out_msg)
        except Exception as e:
            self.get_logger().error(
                f"🔴 Annotation error: {e}", throttle_duration_sec=5.0)

    # -----------------------------------------------------------------------
    # HELPER METHODS
    # -----------------------------------------------------------------------

    def _prepare_image_bytes(self, ros_msg: Image) -> bytes:
        """ROS Image → JPEG bytes via OpenCV (fast path)."""
        image_bgr = self.bridge.imgmsg_to_cv2(ros_msg, 'bgr8')
        ok, buf = cv2.imencode('.jpg', image_bgr,
                               [int(cv2.IMWRITE_JPEG_QUALITY), 95])
        if not ok:
            self.get_logger().error("🔴 Failed to encode image to JPEG!")
            return b""
        return buf.tobytes()

    def _build_annotated_image(self) -> np.ndarray:
        """Converts the latest ROS image and draws current detections on it."""
        image_bgr = self.bridge.imgmsg_to_cv2(
            self.latest_color_msg, "bgr8").copy()
        img_h, img_w = image_bgr.shape[:2]
        with self._detection_lock:
            current_detections = list(self.last_detections)
        return draw_all_annotations(image_bgr, current_detections,
                                    img_w, img_h)

    def _call_lm_studio(
        self,
        img_bytes: bytes,
        user_prompt: Optional[str] = None
    ) -> Optional[List[ObjectDetection]]:
        """Sends image + prompt to LM Studio and parses the JSON response."""
        start       = time.time()
        prompt_text = build_prompt(user_prompt)

        if user_prompt:
            self.get_logger().info("🅱️  User Prompt Mode")
        else:
            self.get_logger().info("🅰️  Default Mode")

        # Base64-encode the JPEG
        image_b64 = base64.b64encode(img_bytes).decode("utf-8")
        image_url = f"data:image/jpeg;base64,{image_b64}"

        messages = [
            {"role": "system", "content": SYSTEM_PROMPT},
            {
                "role": "user",
                "content": [
                    {"type": "image_url",
                     "image_url": {"url": image_url}},
                    {"type": "text",
                     "text": prompt_text},
                ],
            },
        ]

        # Structured JSON output (LM Studio ≥ 0.3)
        response_format = {
            "type": "json_schema",
            "json_schema": {
                "name":   "DetectionResult",
                "strict": True,
                "schema": DetectionResult.model_json_schema(),
            },
        }

        self.get_logger().info(f"📤 Prompt: {prompt_text.strip()}")

        try:
            resp = self.lm_client.chat.completions.create(
                model=self.lm_studio_model,
                messages=messages,
                temperature=LM_STUDIO_TEMPERATURE,
                # max_tokens intentionally omitted → model maximum is used
                response_format=response_format,
            )

            raw_text = resp.choices[0].message.content
            self.get_logger().info(
                f"\n{'=' * 60}\nAnswer:\n{raw_text}\n{'=' * 60}")
            self.get_logger().info(
                f"⏱️  Processing time: {time.time() - start:.2f}s")

            result = DetectionResult.model_validate_json(raw_text)
            return result.objects

        except openai.APIConnectionError as e:
            host = self.get_parameter('lm_studio_host').value
            port = self.get_parameter('lm_studio_port').value
            self.get_logger().error(
                f"🔴 LM Studio not reachable (http://{host}:{port}/v1): {e}\n"
                "   → Is LM Studio running and is a model loaded?"
            )
            return None

        except openai.BadRequestError as e:
            self.get_logger().error(
                f"🔴 BadRequest – model may not support vision input: {e}"
            )
            return None

        except Exception as e:
            self.get_logger().error(f"🔴 LM Studio error: {e}")
            return None

    # -----------------------------------------------------------------------
    # 3D DETECTION PROCESSING  (identical logic to Gemini node)
    # -----------------------------------------------------------------------

    def _process_3d_detections(
        self,
        detections: List[ObjectDetection],
        cv_depth: np.ndarray,
        img_w: int,
        img_h: int,
    ) -> List[DetectedObject]:
        """Projects 2D detections to 3D, builds ROS messages, broadcasts TFs."""
        target_object_msgs: List[DetectedObject] = []
        object_counter: dict = {}

        try:
            tf_cam_to_base = self.tf_buffer.lookup_transform(
                self.robot_base_frame, self.camera_frame,
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=1.0),
            )
        except Exception as e:
            self.get_logger().error(f"TF lookup failed: {e}")
            return []

        for obj in detections:
            color = obj.label.lower()
            ymin, xmin, ymax, xmax = normalized_to_pixel(
                obj.box_2d, img_w, img_h)

            # Yaw from aspect ratio
            w = xmax - xmin
            h = ymax - ymin
            obj.yaw_degrees = (
                0.0 if (h > 0 and 0 < w / h < 1.41) else 30.0)

            depth_mm = get_robust_depth(obj.box_2d, cv_depth, img_w, img_h)
            if depth_mm is None:
                continue

            z = depth_mm / 1000.0 + self.object_center_offset
            if not (0.03 < z <= self.max_depth_m):
                continue

            cx = int(np.clip((xmin + xmax) // 2, 0, img_w - 1))
            cy = int(np.clip((ymin + ymax) // 2, 0, img_h - 1))
            ray   = self.camera_model.projectPixelTo3dRay((cx, cy))
            scale = z / ray[2]

            pt_cam           = PointStamped()
            pt_cam.header.frame_id = self.camera_frame
            pt_cam.point.x   = ray[0] * scale
            pt_cam.point.y   = ray[1] * scale
            pt_cam.point.z   = z
            pt_base          = do_transform_point(pt_cam, tf_cam_to_base)
            obj.position_3d  = [pt_base.point.x,
                                 pt_base.point.y,
                                 pt_base.point.z]

            self.get_logger().info(
                f"DEBUG {color}: pixel=({cx},{cy}), "
                f"ray=({ray[0]:.4f},{ray[1]:.4f},{ray[2]:.4f}), "
                f"scale={scale:.4f}, "
                f"cam=({ray[0]*scale:.4f},{ray[1]*scale:.4f},{z:.4f}), "
                f"base=({pt_base.point.x:.4f},"
                f"{pt_base.point.y:.4f},{pt_base.point.z:.4f})"
            )

            msg                   = DetectedObject()
            msg.color.data        = color
            msg.position          = pt_base
            msg.camera_distance_mm = z * 1000.0
            msg.yaw_degrees       = obj.yaw_degrees
            msg.bounding_box_px   = [ymin, xmin, ymax, xmax]
            msg.has_user_dropoff  = False

            # --- Visual drop-off (2D point) ---
            if obj.dropoff_point_2d:
                drop_mm = get_robust_depth(
                    obj.dropoff_point_2d, cv_depth, img_w, img_h)
                if drop_mm is None:
                    self.get_logger().warn(
                        f"No depth at dropoff for '{color}'. "
                        "Using object depth as fallback.")
                    drop_mm = depth_mm

                drop_z             = drop_mm / 1000.0
                dy, dx             = normalized_to_pixel(
                    obj.dropoff_point_2d, img_w, img_h)
                drop_ray           = self.camera_model.projectPixelTo3dRay(
                    (dx, dy))
                drop_scale         = drop_z / drop_ray[2]

                drop_cam           = PointStamped()
                drop_cam.header.frame_id = self.camera_frame
                drop_cam.point.x   = drop_ray[0] * drop_scale
                drop_cam.point.y   = drop_ray[1] * drop_scale
                drop_cam.point.z   = drop_z
                drop_base          = do_transform_point(drop_cam, tf_cam_to_base)

                obj.dropoff_3d            = [drop_base.point.x,
                                             drop_base.point.y,
                                             drop_base.point.z]
                msg.has_user_dropoff      = True
                msg.user_dropoff_position = drop_base.point

            # --- Metric drop-off (explicit x/y from prompt) ---
            elif obj.dropoff_coords_m:
                msg.has_user_dropoff          = True
                msg.user_dropoff_position.x   = float(obj.dropoff_coords_m[0])
                msg.user_dropoff_position.y   = float(obj.dropoff_coords_m[1])
                msg.user_dropoff_position.z   = pt_base.point.z
                obj.dropoff_3d                = [
                    msg.user_dropoff_position.x,
                    msg.user_dropoff_position.y,
                    msg.user_dropoff_position.z,
                ]

            target_object_msgs.append(msg)

            # --- TF broadcast ---
            count                = object_counter.get(color, 0)
            object_counter[color] = count + 1
            child_frame          = f"object_{color}_{count}"

            t                    = TransformStamped()
            t.header.stamp       = self.get_clock().now().to_msg()
            t.header.frame_id    = self.robot_base_frame
            t.child_frame_id     = child_frame
            t.transform.translation.x = pt_base.point.x
            t.transform.translation.y = pt_base.point.y
            t.transform.translation.z = pt_base.point.z
            yaw_rad              = math.radians(obj.yaw_degrees)
            t.transform.rotation.z = math.sin(yaw_rad / 2.0)
            t.transform.rotation.w = math.cos(yaw_rad / 2.0)
            self.tf_broadcaster.sendTransform(t)

        return target_object_msgs


# ---------------------------------------------------------------------------
# MAIN
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node     = LmStudioVisionNode()
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