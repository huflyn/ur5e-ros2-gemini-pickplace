#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

def nothing(x):  # dummy function for trackbar callback
    pass

# How to start: 
# ros2 run color_detection hsv_tuner --ros-args -p color_image_topic:=/webots_realsense/color/image_raw/image_color

class HSVTunerNode(Node):
    def __init__(self):
        super().__init__('hsv_tuner_node')
        self.bridge = CvBridge()
        
        # Declare parameters with defaults (can be overridden via command line or launch file)
        self.declare_parameter('color_image_topic', '/camera/color/image_raw')
        topic = self.get_parameter('color_image_topic').get_parameter_value().string_value
        
        # subscriber
        self.create_subscription(Image, topic, self.image_callback, 10)
        
        # OpenCV window and trackbars
        cv2.namedWindow('Trackbars')
        cv2.createTrackbar('HMin', 'Trackbars', 0, 179, nothing)
        cv2.createTrackbar('SMin', 'Trackbars', 0, 255, nothing)
        cv2.createTrackbar('VMin', 'Trackbars', 0, 255, nothing)
        cv2.createTrackbar('HMax', 'Trackbars', 179, 179, nothing)
        cv2.createTrackbar('SMax', 'Trackbars', 255, 255, nothing)
        cv2.createTrackbar('VMax', 'Trackbars', 255, 255, nothing)
        
        # Timer that prints the values in a clean format to the terminal
        self.create_timer(2.0, self.print_yaml_format)
        
        self.get_logger().info(f"HSV Tuner started. Listening on: {topic}")

    def print_yaml_format(self):
        hMin = cv2.getTrackbarPos('HMin', 'Trackbars')
        sMin = cv2.getTrackbarPos('SMin', 'Trackbars')
        vMin = cv2.getTrackbarPos('VMin', 'Trackbars')
        hMax = cv2.getTrackbarPos('HMax', 'Trackbars')
        sMax = cv2.getTrackbarPos('SMax', 'Trackbars')
        vMax = cv2.getTrackbarPos('VMax', 'Trackbars')
        
        yaml_output = (
            f"\n"
            f"----------------------------------------\n"
            f"--- Copy this into your YAML file ---\n"
            f"hsv_lower: [{hMin}, {sMin}, {vMin}]\n"
            f"hsv_upper: [{hMax}, {sMax}, {vMax}]\n"
            f"----------------------------------------"
        )
        self.get_logger().info(yaml_output)
        

    def image_callback(self, img_msg):
        frame = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        
        hMin = cv2.getTrackbarPos('HMin', 'Trackbars')
        sMin = cv2.getTrackbarPos('SMin', 'Trackbars')
        vMin = cv2.getTrackbarPos('VMin', 'Trackbars')
        hMax = cv2.getTrackbarPos('HMax', 'Trackbars')
        sMax = cv2.getTrackbarPos('SMax', 'Trackbars')
        vMax = cv2.getTrackbarPos('VMax', 'Trackbars')
        
        lower = np.array([hMin, sMin, vMin])
        upper = np.array([hMax, sMax, vMax])
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower, upper)
        result = cv2.bitwise_and(frame, frame, mask=mask)
        
        cv2.imshow('Original + Masked', result)
        cv2.imshow('Pure Mask (Black/White)', mask) # B/W mask for better visualization
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = HSVTunerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Tuner stopped.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()