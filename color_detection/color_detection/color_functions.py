import numpy as np
import cv2

def detect_color(hsv, color, color_bounds):
    """
    Creates a binary mask for a given color using provided HSV bounds.
    """
    if color in color_bounds:
        lower = color_bounds[color]['lower']
        upper = color_bounds[color]['upper']
        mask = cv2.inRange(hsv, lower, upper)
        return mask
    else:
        # Return an empty mask if the color is not defined in the bounds
        return np.zeros(hsv.shape[:2], dtype=np.uint8)

def process_mask(mask):
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=1)
    mask = cv2.dilate(mask, kernel, iterations=1)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return contours

   
def display_information(cv2_img, color, x, y, w, h, center_x, center_y, depth_image, logger=None):
    if depth_image is not None:
        depth = depth_image[center_y, center_x]
        if depth > 1000:  # Ignore objects farther than 1000mm
            return
            
        # Use ROS logger if provided, otherwise force print flush
        if logger:
            logger.info(f"Camera lens -> {color} brick: {depth:.1f} mm")
        else:
            print(f"Camera lens -> {color} brick: {depth:.1f} mm", flush=True)
    
    cv2.rectangle(cv2_img, (x, y), (x+w, y+h), (0, 255, 0), 2)
    cv2.putText(cv2_img, color, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0,255,0), 2)