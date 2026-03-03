#!/usr/bin/env python3

import cv2
import numpy as np
import random

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped

import tf2_ros
import tf2_geometry_msgs
from image_geometry import PinholeCameraModel

# Eigene MSG und Funktionen
from color_detection_msgs.msg import LegoBrick
from color_detection.color_functions import detect_color, process_mask, display_information


class ColorDetectorNode(Node):
    def __init__(self):
        super().__init__('color_detector_node')
        
        # --- Variablen ---
        self.bridge = CvBridge()
        self.camera_model = PinholeCameraModel()
        self.depth_image = None
        self.detected_lego_bricks = []
        self.image_processed = False
        
        # --- TF2 Setup ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # --- Publisher ---
        self.lego_brick_pub = self.create_publisher(LegoBrick, '/lego_brick_info', 10)
        self.lego_brick_coords_pub = self.create_publisher(PointStamped, '/lego_brick_coords', 10)
        self.lego_brick_color_pub = self.create_publisher(String, '/lego_brick_color', 10)

        # --- Parameters ---
        # --- Declare basic parameters ---
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('depth_image_topic', '/camera/depth/image_raw')
        self.declare_parameter('color_image_topic', '/camera/color/image_raw')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('robot_base_frame', 'base_link')        
        self.declare_parameter('sort_method', 'y_axis') # Options: 'y_axis' (default, deterministic) or 'random' (prevents endless loops on edge cases)
        self.declare_parameter('verbose', False)

        # --- Read basic parameters ---
        info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        depth_topic = self.get_parameter('depth_image_topic').get_parameter_value().string_value
        color_topic = self.get_parameter('color_image_topic').get_parameter_value().string_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        self.robot_base_frame = self.get_parameter('robot_base_frame').get_parameter_value().string_value

        self.get_logger().info(f"Listening to color topic: {color_topic}")
        self.get_logger().info(f"Using camera frame: {self.camera_frame}")
        self.get_logger().info(f"Transforming coordinates to: {self.robot_base_frame}")

        # --- Declare HSV parameters (with default fallbacks) ---
        self.declare_parameter('hsv_red_lower', [0, 200, 0])
        self.declare_parameter('hsv_red_upper', [10, 255, 130])
        self.declare_parameter('hsv_yellow_lower', [17, 170, 0])
        self.declare_parameter('hsv_yellow_upper', [23, 255, 255])
        self.declare_parameter('hsv_green_lower', [60, 90, 0])
        self.declare_parameter('hsv_green_upper', [90, 255, 255])
        self.declare_parameter('hsv_blue_lower', [90, 150, 55])
        self.declare_parameter('hsv_blue_upper', [105, 255, 80])

        # --- Read HSV parameters and structure them in a dictionary ---
        self.color_bounds = {
            'red': {
                'lower': np.array(self.get_parameter('hsv_red_lower').value),
                'upper': np.array(self.get_parameter('hsv_red_upper').value)
            },
            'yellow': {
                'lower': np.array(self.get_parameter('hsv_yellow_lower').value),
                'upper': np.array(self.get_parameter('hsv_yellow_upper').value)
            },
            'green': {
                'lower': np.array(self.get_parameter('hsv_green_lower').value),
                'upper': np.array(self.get_parameter('hsv_green_upper').value)
            },
            'blue': {
                'lower': np.array(self.get_parameter('hsv_blue_lower').value),
                'upper': np.array(self.get_parameter('hsv_blue_upper').value)
            }
        }
        
        # --- Subscriber (mit den dynamischen Variablen) ---
        self.create_subscription(CameraInfo, info_topic, self.camera_info_callback, 10)
        self.create_subscription(Image, depth_topic, self.depth_callback, 10)
        self.create_subscription(Image, color_topic, self.image_callback, 10)
        
        # --- Timer für das Senden ---
        # Führt die Funktion alle 1.0 Sekunden aus
        self.publish_timer = self.create_timer(1.0, self.publish_bricks_timer_callback)
        
        self.get_logger().info("Color Detector Node (ROS 2) successfully started.") 


    def camera_info_callback(self, data):
        self.camera_model.fromCameraInfo(data)


    def depth_callback(self, data):
        try:
            # Automatische Unterscheidung zwischen Simulation und echter Hardware
            if data.encoding == '32FC1':
                cv_image_meters = self.bridge.imgmsg_to_cv2(data, desired_encoding="32FC1")
                self.depth_image = cv_image_meters * 1000.0 # Umrechnung in Millimeter
            else:
                self.depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="16UC1")
        except CvBridgeError as e:
            self.get_logger().error(f"Error converting depth image: {e}")


    def transform_point(self, x, y, z):
        point_in_camera_frame = PointStamped()
        
        point_in_camera_frame.header.frame_id = self.camera_frame
        point_in_camera_frame.header.stamp = self.get_clock().now().to_msg()
        
        point_in_camera_frame.point.x = x / 1000.0
        point_in_camera_frame.point.y = y / 1000.0
        point_in_camera_frame.point.z = z / 1000.0

        try:
            # Calculate the path from the optical frame to the robot's base frame
            transform = self.tf_buffer.lookup_transform(self.robot_base_frame, self.camera_frame, rclpy.time.Time())
            
            # Transform the point
            point_in_target_frame = tf2_geometry_msgs.do_transform_point(point_in_camera_frame, transform)
            return point_in_target_frame
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f"TF Error: {e}")
            return None


    def is_duplicate(self, new_point, new_color, existing_bricks, position_threshold=0.1):
        # Update: existing_bricks now contains tuples of 3 elements (point, color, depth)
        for brick_data in existing_bricks:
            existing_point = brick_data[0]
            existing_color = brick_data[1]
            
            distance = np.sqrt((new_point.point.x - existing_point.point.x) ** 2 +
                               (new_point.point.y - existing_point.point.y) ** 2 +
                               (new_point.point.z - existing_point.point.z) ** 2)
            if distance < position_threshold and new_color == existing_color:
                return True
        return False


    def image_callback(self, img_msg):
        if self.image_processed:
            return
        self.image_processed = True

        try:
            cv2_img = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert img_msg to cv2: {e}")
            return

        hsv = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2HSV)
        colors = ['green', 'yellow', 'red', 'blue']

        img_h, img_w = cv2_img.shape[:2] # Get image dimensions for edge margin calculation
        edge_margin = 25 # Margin in pixels to ignore detections near the edges

        for color in colors:
            mask = detect_color(hsv, color, self.color_bounds)
            contours = process_mask(mask)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                rel_area = area / (cv2_img.shape[0] * cv2_img.shape[1])

                if rel_area > 0.001:
                    x, y, w, h = cv2.boundingRect(cnt)

                    # --- Safe Zone Check: Ignore detections near the edges ---
                    if (x < edge_margin or y < edge_margin or 
                        x + w > img_w - edge_margin or y + h > img_h - edge_margin):
                        continue  # Skip this detection as it's too close to the edge

                    center_x = x + w // 2
                    center_y = y + h // 2
                    
                    # Fetch the depth distance silently
                    brick_depth = display_information(cv2_img, color, x, y, w, h, center_x, center_y, self.depth_image, self.get_logger())

                    if brick_depth is not None:
                        if self.camera_model.fx() and self.camera_model.fy():
                            ray = self.camera_model.projectPixelTo3dRay((center_x, center_y))
                            transformed_point = self.transform_point(ray[0] * brick_depth, ray[1] * brick_depth, brick_depth)
                            
                            if transformed_point is not None:
                                if not self.is_duplicate(transformed_point, color, self.detected_lego_bricks):
                                    # Append ALL 3 values to the list
                                    self.detected_lego_bricks.append((transformed_point, color, brick_depth))

        # Draw a red rectangle to indicate the safe zone (optional, for debugging)
        cv2.rectangle(cv2_img, (edge_margin, edge_margin), (img_w - edge_margin, img_h - edge_margin), (0, 0, 255), 2)

        # Display the image with detections (optional, for debugging)
        cv2.imshow('Color Detection', cv2_img)
        cv2.waitKey(1)


    def publish_bricks_timer_callback(self):
        """Called by the timer to publish the detected bricks silently."""
        if self.detected_lego_bricks:
            
            # Fetch the current sorting method dynamically
            sort_method = self.get_parameter('sort_method').get_parameter_value().string_value
            
            if sort_method == 'random':
                # Shuffle the list randomly to break deterministic endless loops on unreachable bricks
                random.shuffle(self.detected_lego_bricks)
                sorted_bricks = self.detected_lego_bricks
            else:
                # Default fallback: Closest - Sort by Y-coordinate
                sorted_bricks = sorted(self.detected_lego_bricks, key=lambda brick: brick[0].point.y)

            # Fetch the current logging state dynamically
            verbose = self.get_parameter('verbose').get_parameter_value().bool_value
            
            # Unpack all 3 values
            for brick_point, brick_color, brick_depth in sorted_bricks:
                lego_brick_msg = LegoBrick()
                lego_brick_msg.position = brick_point
                lego_brick_msg.color.data = brick_color
                lego_brick_msg.camera_distance_mm = brick_depth

                if verbose:
                    self.get_logger().info(f"Debug: Publishing {brick_color.upper()} brick at X:{brick_point.point.x:.3f}, Y:{brick_point.point.y:.3f}, Depth:{brick_depth}")
                
                self.lego_brick_pub.publish(lego_brick_msg)
                
            self.detected_lego_bricks.clear()
        
        # Reset flag for the next camera frame
        self.image_processed = False


def main(args=None):
    rclpy.init(args=args)
    node = ColorDetectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node manually stopped.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()