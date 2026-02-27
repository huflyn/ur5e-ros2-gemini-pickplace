import numpy as np
import cv2

def detect_color(hsv, color, color_bounds):
    """
    Creates a binary mask for a given color using provided HSV bounds.
    Special handling for 'red' due to its split nature in the HSV color space (H=0-10 and H=170-179).
    """
    if color not in color_bounds:
        # Return an empty mask if the color is not defined in the bounds
        return np.zeros(hsv.shape[:2], dtype=np.uint8)

    if color == 'red':
        # RED: Requires two masks because the hue wraps around 180 in OpenCV
        
        # 1. Lower Red Mask (e.g., 0-10)
        lower_red_1 = color_bounds['red']['lower']
        upper_red_1 = color_bounds['red']['upper']
        mask1 = cv2.inRange(hsv, lower_red_1, upper_red_1)
        
        # 2. Upper Red Mask (e.g., 170-179). We reuse Saturation and Value bounds from the first.
        # We assume the user tuned the lower hue perfectly, so we mirror it to the top.
        lower_red_2 = np.array([170, lower_red_1[1], lower_red_1[2]])
        upper_red_2 = np.array([180, upper_red_1[1], upper_red_1[2]])
        mask2 = cv2.inRange(hsv, lower_red_2, upper_red_2)
        
        # Combine both masks
        mask = cv2.bitwise_or(mask1, mask2)
        return mask
        
    else:
        # STANDARD COLORS (Blue, Green, Yellow)
        lower = color_bounds[color]['lower']
        upper = color_bounds[color]['upper']
        mask = cv2.inRange(hsv, lower, upper)
        return mask

def process_mask(mask):
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=1)
    mask = cv2.dilate(mask, kernel, iterations=1)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return contours

   
def display_information(cv2_img, color, x, y, w, h, center_x, center_y, depth_image, logger=None):
    """
    Extracts the depth information at the center of the detected object.
    Draws bounding boxes and returns the depth in mm.
    """
    depth_val = None
    if depth_image is not None:
        depth = float(depth_image[center_y, center_x])
        if depth <= 1000:  # Ignore objects farther than 1000mm
            depth_val = depth
            
    # Draw bounding box and text (Keep these so your cv2.imshow still looks good!)
    cv2.rectangle(cv2_img, (x, y), (x+w, y+h), (0, 255, 0), 2)
    cv2.putText(cv2_img, color, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0,255,0), 2)
    
    return depth_val