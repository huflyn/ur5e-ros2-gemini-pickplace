#!/usr/bin/env python3

import cv2
import numpy as np
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, TransformStamped
from cv_bridge import CvBridge, CvBridgeError
import message_filters

import tf2_ros
import tf2_geometry_msgs
from image_geometry import PinholeCameraModel

# Eigene MSG, SRV und Funktionen
from brick_interfaces.msg import LegoBrick
from brick_interfaces.srv import DetectBricks
from color_detection.color_functions import detect_color, process_mask


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


class ColorDetectorNode(Node):
    def __init__(self):
        super().__init__('color_detector_node')
        
        # --- Variables ---
        self.bridge = CvBridge()
        self.camera_model = PinholeCameraModel()
        self.camera_info_ready = False
        
        self.latest_color_msg = None
        self.latest_depth_msg = None
        
        # --- TF2 Setup ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- TF2 Broadcaster ---
        # Used to publish the 3D position of the bricks to RViz
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # --- Parameters ---
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('depth_image_topic', '/camera/depth/image_raw')
        self.declare_parameter('color_image_topic', '/camera/color/image_raw')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('robot_base_frame', 'base_link')        
        self.declare_parameter('brick_center_offset', 0.008) # in m. Offset to reach brick center instead of front face

        info_topic = self.get_parameter('camera_info_topic').value
        depth_topic = self.get_parameter('depth_image_topic').value
        color_topic = self.get_parameter('color_image_topic').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.robot_base_frame = self.get_parameter('robot_base_frame').value

        # --- Declare HSV parameters ---
        self.declare_parameter('hsv_red_lower', [0, 200, 0])
        self.declare_parameter('hsv_red_upper', [10, 255, 130])
        self.declare_parameter('hsv_yellow_lower', [17, 170, 0])
        self.declare_parameter('hsv_yellow_upper', [23, 255, 255])
        self.declare_parameter('hsv_green_lower', [60, 90, 0])
        self.declare_parameter('hsv_green_upper', [90, 255, 255])
        self.declare_parameter('hsv_blue_lower', [90, 150, 55])
        self.declare_parameter('hsv_blue_upper', [105, 255, 80])

        self.color_bounds = {
            'red': {'lower': np.array(self.get_parameter('hsv_red_lower').value), 'upper': np.array(self.get_parameter('hsv_red_upper').value)},
            'yellow': {'lower': np.array(self.get_parameter('hsv_yellow_lower').value), 'upper': np.array(self.get_parameter('hsv_yellow_upper').value)},
            'green': {'lower': np.array(self.get_parameter('hsv_green_lower').value), 'upper': np.array(self.get_parameter('hsv_green_upper').value)},
            'blue': {'lower': np.array(self.get_parameter('hsv_blue_lower').value), 'upper': np.array(self.get_parameter('hsv_blue_upper').value)}
        }
        
        # --- Subscribers (Synchronized) ---
        self.create_subscription(CameraInfo, info_topic, self.camera_info_callback, 10)
        
        self.color_sub = message_filters.Subscriber(self, Image, color_topic)
        self.depth_sub = message_filters.Subscriber(self, Image, depth_topic)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub], queue_size=5, slop=0.1
        )
        self.ts.registerCallback(self.image_sync_callback)
        
        # --- Service Server ---
        self.detect_srv = self.create_service(
            DetectBricks, '/detect_bricks', self.detect_callback
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
            "🟢 ColorDetectorNode (Service Node) ready.\n" +
            "👂 Waiting for service call from client...\n" +
            "To test functionality, you can call the service in a new terminal:\n" +
            "ros2 service call /detect_bricks brick_interfaces/srv/DetectBricks\n" +
            "="*60
        )

    # --- Callbacks ---

    def camera_info_callback(self, data):
        # Print the log only once when the first message arrives
        if not self.camera_info_ready:
            self.get_logger().info("Camera intrinsics received.")
            self.camera_info_ready = True
            
        # Safely feed the data into the model
        self.camera_model.fromCameraInfo(data)
            
    def image_sync_callback(self, color_msg, depth_msg):
        """Continually caches the latest synchronized image pair."""
        self.latest_color_msg = color_msg
        self.latest_depth_msg = depth_msg

    # --- Helper Functions ---

    def transform_point(self, x, y, z):
        point_in_camera_frame = PointStamped()
        point_in_camera_frame.header.frame_id = self.camera_frame
        point_in_camera_frame.header.stamp = self.get_clock().now().to_msg()
        point_in_camera_frame.point.x = x / 1000.0
        point_in_camera_frame.point.y = y / 1000.0
        point_in_camera_frame.point.z = z / 1000.0

        try:
            transform = self.tf_buffer.lookup_transform(self.robot_base_frame, self.camera_frame, rclpy.time.Time())
            return tf2_geometry_msgs.do_transform_point(point_in_camera_frame, transform)
        except Exception as e:
            self.get_logger().warn(f"TF Error: {e}")
            return None


    def is_duplicate(self, new_point, new_color, existing_bricks, position_threshold=0.1):
        for brick_msg in existing_bricks:
            if new_color == brick_msg.color.data:
                dist = np.sqrt(
                    (new_point.point.x - brick_msg.position.point.x)**2 +
                    (new_point.point.y - brick_msg.position.point.y)**2 +
                    (new_point.point.z - brick_msg.position.point.z)**2
                )
                if dist < position_threshold:
                    return True
        return False

    def publish_vis_timer(self):
        """Continuously publishes the last annotated image for rqt_image_view."""
        if self.latest_annotated_msg is not None:
            # Update the timestamp to prevent RQT from ignoring "old" messages
            self.latest_annotated_msg.header.stamp = self.get_clock().now().to_msg()
            self.annotated_pub.publish(self.latest_annotated_msg)

    # --- Service Handler ---
    def detect_callback(self, request, response):
        """Triggered by the orchestrator to perform an on-demand scan.
          ros2 service call /detect_bricks brick_interfaces/srv/DetectBricks
        """
        self.get_logger().info("Service /detect_bricks called (Color-Detector).")

        if self.latest_color_msg is None or self.latest_depth_msg is None:
            response.success = False
            response.error_message = "No images received yet"
            return response

        # --- Camera Info check ---
        if not self.camera_info_ready:
            response.success = False
            response.error_message = "Camera info not yet received"
            return response

        try:
            cv_color = self.bridge.imgmsg_to_cv2(self.latest_color_msg, "bgr8")
            
            # Depth handling
            if self.latest_depth_msg.encoding == '32FC1':
                depth_meters = self.bridge.imgmsg_to_cv2(self.latest_depth_msg, desired_encoding="32FC1")
                cv_depth = depth_meters * 1000.0 # to mm
            else:
                cv_depth = self.bridge.imgmsg_to_cv2(self.latest_depth_msg, desired_encoding="16UC1")
                
        except CvBridgeError as e:
            response.success = False
            response.error_message = f"CV Bridge Error: {e}"
            return response

        hsv = cv2.cvtColor(cv_color, cv2.COLOR_BGR2HSV)
        colors = ['green', 'yellow', 'red', 'blue']
        img_h, img_w = cv_color.shape[:2]

        # Define a safe margin to avoid edge artifacts (tunable based on your setup)
        edge_margin_x = 260
        edge_margin_y = 1
        
        # Add brick width offset to ensure we target the center of the brick rather than the front face
        brick_center_offset = self.get_parameter('brick_center_offset').value
        detected_bricks = []

        # Find bricks using HSV masks
        for color in colors:
            mask = detect_color(hsv, color, self.color_bounds)
            contours = process_mask(mask)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                rel_area = area / (cv_color.shape[0] * cv_color.shape[1])

                if rel_area > 0.001:
                    x, y, w, h = cv2.boundingRect(cnt)

                    # Calculate aspect ratio based on horizontal camera perspective
                    aspect_ratio = w / h if h > 0 else 0.0

                    # Orientation logic for 4x2 bricks from side-view:
                    if aspect_ratio >= 1.41:
                        yaw_degrees = 0.0 # Brick is horizontal (parallel to camera plane)
                    elif aspect_ratio > 0:
                        yaw_degrees = 30.0 # Brick is vertical (pointing away from camera)
                    else:
                        yaw_degrees = 0.0 # Fallback for invalid detections

                    # Skip contours that are too close to the edges to avoid false detections
                    if (x < edge_margin_x or y < edge_margin_y or 
                        x + w > img_w - edge_margin_x or y + h > img_h - edge_margin_y):
                        continue

                    center_x = x + w // 2
                    center_y = y + h // 2
                    
                    # --- ROBUST MEDIAN DEPTH CALCULATION ---
                    # Extract a 7x7 pixel region around the center
                    region = cv_depth[
                        max(0, center_y - 3):min(img_h, center_y + 4),
                        max(0, center_x - 3):min(img_w, center_x + 4)
                    ]
                    
                    # Filter out 0 and inf/NaN values
                    valid_depth = region[(region > 0) & np.isfinite(region)]

                    if valid_depth.size == 0:
                        self.get_logger().warn(f"No valid depth for {color} brick")
                        cv2.putText(cv_color, f"{color} (no depth)",
                                    (x, max(20, y - 10)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 2)
                        continue

                    # Get the median depth (filters out studs and noise)
                    raw_front_depth = float(np.median(valid_depth))

                    # Apply the offset to reach the physical center of the brick
                    brick_depth = raw_front_depth + (brick_center_offset * 1000.0)
                    
                    ray = self.camera_model.projectPixelTo3dRay((center_x, center_y))
                    transformed_point = self.transform_point(ray[0] * brick_depth, ray[1] * brick_depth, brick_depth)
                    
                    if transformed_point is not None:
                        if not self.is_duplicate(transformed_point, color, detected_bricks):
                            
                            msg = LegoBrick()
                            msg.position = transformed_point
                            msg.color.data = color
                            msg.camera_distance_mm = float(brick_depth)
                            msg.yaw_degrees = yaw_degrees
                            msg.bounding_box_px = [x, y, x+w, y+h]
                            
                            detected_bricks.append(msg)

                            # --- Annotation ---
                            draw_color = (255, 255, 255) # Weiß für die Annotation
                            draw_brick_annotation(cv_color, color, x, y, x+w, y+h, transformed_point.point.x, transformed_point.point.y, draw_color)

                            self.get_logger().info(
                                f"""{color:>8s}: X={transformed_point.point.x:.3f}, Y={transformed_point.point.y:.3f}, Z={transformed_point.point.z:.3f}, 
                                Aspect Ratio={aspect_ratio:.2f}, Yaw={yaw_degrees:.2f} degrees, Depth={brick_depth:.1f}mm"""
                            )

        # Draw safe zone
        cv2.rectangle(cv_color, (edge_margin_x, edge_margin_y), (img_w - edge_margin_x, img_h - edge_margin_y), (0, 0, 0), 4)
        cv2.rectangle(cv_color, (edge_margin_x, edge_margin_y), (img_w - edge_margin_x, img_h - edge_margin_y), (0, 0, 255), 1)
        
        # Construct and send response
        response.success = True
        response.error_message = ""
        response.bricks = detected_bricks

        # --- Broadcast TF Frames for RViz ---
        # Iterate through the final list of detected bricks and send a TF for each
        brick_counter = {}
        for brick in detected_bricks:
            color = brick.color.data
            
            # Track counts to create unique TF names (e.g., 'brick_red_0')
            count = brick_counter.get(color, 0)
            brick_counter[color] = count + 1
            
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.robot_base_frame
            t.child_frame_id = f"brick_{color}_{count}"
            
            # Apply 3D position
            t.transform.translation.x = brick.position.point.x
            t.transform.translation.y = brick.position.point.y
            t.transform.translation.z = brick.position.point.z
            
            # Convert yaw (degrees) to quaternion for Z-axis rotation
            yaw_rad = math.radians(brick.yaw_degrees)
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = math.sin(yaw_rad / 2.0)
            t.transform.rotation.w = math.cos(yaw_rad / 2.0)
            
            # Publish to the tf tree
            self.tf_broadcaster.sendTransform(t)
        # ------------------------------------
        
        # Publish annotated image as ROS topic
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(cv_color, encoding='bgr8')
            annotated_msg.header.stamp = self.get_clock().now().to_msg()
            annotated_msg.header.frame_id = self.camera_frame
            
            # --- Save the message for the continuous timer ---
            self.latest_annotated_msg = annotated_msg
            
            self.annotated_pub.publish(annotated_msg)
            self.get_logger().info("Annotated image published on /annotated_image")
        except Exception as e:
            self.get_logger().error(f"Failed to publish annotated image: {e}")

        self.get_logger().info(f"Returning {len(detected_bricks)} brick(s). Service call complete.")

        self.get_logger().info(
            "Status:"
            "\n" + "="*60 + "\n" +
            "ColorDetectorNode (Service Mode) ready. Waiting for new /detect_bricks call...\n" +
            "="*60
        )

        return response


def main(args=None):
    rclpy.init(args=args)
    node = ColorDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()