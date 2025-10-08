"""
Camera module for the Soccer Robot
Handles Pi Camera initialization, frame capture, and ball detection
"""

import cv2
import numpy as np
import math
from picamera2 import Picamera2
from config import CAMERA_CONFIG, BALL_DETECTION, GOAL_DETECTION, CIRCULAR_MASK, FRAME_CONFIG


class Camera:
    """Handles camera operations and ball detection for the soccer robot"""
    
    def __init__(self):
        """Initialize the Pi Camera and detection parameters"""
        # Initialize Pi Camera
        self.picam2 = Picamera2()
        self.picam2.configure(self.picam2.create_video_configuration(
            main={"size": (CAMERA_CONFIG["width"], CAMERA_CONFIG["height"]), 
                  "format": CAMERA_CONFIG["format"]}
        ))
        self.picam2.start()
        
        # Ball detection parameters from config
        self.lower_orange = np.array(BALL_DETECTION["lower_orange"])
        self.upper_orange = np.array(BALL_DETECTION["upper_orange"])
        
        # Circular mask parameters from config
        self.mask_center_x = CIRCULAR_MASK["center_x"]
        self.mask_center_y = CIRCULAR_MASK["center_y"]
        self.mask_radius = CIRCULAR_MASK["radius"]
        
        # Frame configuration
        self.frame_center_x = FRAME_CONFIG["center_x"]
        self.frame_center_y = FRAME_CONFIG["center_y"]
        
        # Ball detection state
        self.ball_center_x = 0
        self.ball_center_y = 0
        self.ball_radius = 0
        self.ball_detected = False
        
        # Ball detection thresholds from config
        self.proximity_threshold = BALL_DETECTION["proximity_threshold"]
        self.angle_tolerance = BALL_DETECTION["angle_tolerance"]
        
        # Goal detection state
        self.blue_goal_detected = False
        self.yellow_goal_detected = False
        self.blue_goal_center_x = 0
        self.blue_goal_center_y = 0
        self.blue_goal_width = 0
        self.blue_goal_height = 0
        self.yellow_goal_center_x = 0
        self.yellow_goal_center_y = 0
        self.yellow_goal_width = 0
        self.yellow_goal_height = 0
        
        # Goal detection parameters from config
        self.blue_goal_config = GOAL_DETECTION["blue_goal"]
        self.yellow_goal_config = GOAL_DETECTION["yellow_goal"]
        self.goal_detection_params = GOAL_DETECTION
    
    def capture_frame(self):
        """Capture a frame from the Pi Camera"""
        return self.picam2.capture_array()
    
    def detect_ball(self, frame):
        """
        Detect orange ball in the frame using HSV color filtering
        
        Args:
            frame: Input frame from camera
            
        Returns:
            tuple: (ball_found, ball_center, ball_radius)
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_orange, self.upper_orange)
        
        # Apply circular mask to ignore center area
        mask = self._apply_circular_mask(mask)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            print(f"Contour Area: {cv2.contourArea(contour)}")

        # Filter contours by area using config values
        filtered_contours = [x for x in contours if 
                           cv2.contourArea(x) > BALL_DETECTION["min_contour_area"] and 
                           cv2.contourArea(x) < BALL_DETECTION["max_contour_area"]]

        contours = filtered_contours

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            (x, y), radius = cv2.minEnclosingCircle(largest_contour)
            
            self.ball_center_x = int(x)
            self.ball_center_y = int(y)
            self.ball_radius = int(radius)
            self.ball_detected = True
            
            return True, (self.ball_center_x, self.ball_center_y), self.ball_radius
        
        self.ball_detected = False
        return False, None, 0
    
    def _apply_circular_mask(self, mask):
        """Apply a circular mask to ignore the center area of the frame"""
        # Create a circular mask (white circle on black background)
        mask_with_circle = np.zeros_like(mask)
        cv2.circle(mask_with_circle, (self.mask_center_x, self.mask_center_y), 
                   self.mask_radius, 255, -1)
        
        # Invert the circle mask (black circle on white background)
        circle_mask = cv2.bitwise_not(mask_with_circle)
        
        # Apply the mask to the original detection mask
        masked_result = cv2.bitwise_and(mask, circle_mask)
        
        return masked_result
    
    def get_ball_proximity_info(self):
        """
        Get ball proximity and centering information
        
        Returns:
            dict: Contains ball area, horizontal/vertical errors, and status flags
        """
        if not self.ball_detected:
            return {
                'ball_area': 0,
                'horizontal_error': 0,
                'vertical_error': 0,
                'is_close': False,
                'is_centered_horizontally': False,
                'is_close_and_centered': False
            }
        
        # Calculate ball area and errors
        ball_area = math.pi * self.ball_radius * self.ball_radius
        horizontal_error = (self.ball_center_x - self.frame_center_x) / self.frame_center_x
        vertical_error = (self.ball_center_y - self.frame_center_y) / self.frame_center_y
        
        # Check proximity and centering
        is_close = ball_area >= self.proximity_threshold
        is_centered_horizontally = abs(horizontal_error) <= self.angle_tolerance
        is_close_and_centered = is_close and is_centered_horizontally
        
        return {
            'ball_area': ball_area,
            'horizontal_error': horizontal_error,
            'vertical_error': vertical_error,
            'is_close': is_close,
            'is_centered_horizontally': is_centered_horizontally,
            'is_close_and_centered': is_close_and_centered
        }
    
    def get_ball_position(self):
        """Get current ball position and detection status"""
        return {
            'detected': self.ball_detected,
            'center_x': self.ball_center_x,
            'center_y': self.ball_center_y,
            'radius': self.ball_radius
        }
    
    def detect_goals(self, frame):
        """
        Detect blue and yellow goals in the frame using HSV color filtering
        
        Args:
            frame: Input frame from camera
            
        Returns:
            dict: Goal detection results for both blue and yellow goals
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Detect blue goal (target goal)
        blue_result = self._detect_single_goal(hsv, self.blue_goal_config, "blue")
        self.blue_goal_detected = blue_result['detected']
        if self.blue_goal_detected:
            self.blue_goal_center_x = blue_result['center_x']
            self.blue_goal_center_y = blue_result['center_y']
            self.blue_goal_width = blue_result['width']
            self.blue_goal_height = blue_result['height']
        
        # Detect yellow goal (opponent goal)
        yellow_result = self._detect_single_goal(hsv, self.yellow_goal_config, "yellow")
        self.yellow_goal_detected = yellow_result['detected']
        if self.yellow_goal_detected:
            self.yellow_goal_center_x = yellow_result['center_x']
            self.yellow_goal_center_y = yellow_result['center_y']
            self.yellow_goal_width = yellow_result['width']
            self.yellow_goal_height = yellow_result['height']
        
        return {
            'blue_goal': blue_result,
            'yellow_goal': yellow_result
        }
    
    def _detect_single_goal(self, hsv_frame, goal_config, goal_type):
        """
        Detect a single goal (blue or yellow) using HSV filtering
        
        Args:
            hsv_frame: HSV converted frame
            goal_config: Configuration for the specific goal type
            goal_type: String identifier for the goal type
            
        Returns:
            dict: Goal detection result
        """
        # Create HSV mask for the goal color
        lower = np.array(goal_config["lower"])
        upper = np.array(goal_config["upper"])
        mask = cv2.inRange(hsv_frame, lower, upper)
        
        # Apply circular mask to ignore center area (same as ball detection)
        mask = self._apply_circular_mask(mask)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        # Filter contours by area
        filtered_contours = [c for c in contours if 
                           cv2.contourArea(c) > goal_config["min_contour_area"] and 
                           cv2.contourArea(c) < goal_config["max_contour_area"]]
        
        if not filtered_contours:
            return {
                'detected': False,
                'center_x': 0,
                'center_y': 0,
                'width': 0,
                'height': 0,
                'area': 0
            }
        
        # Find the largest contour
        largest_contour = max(filtered_contours, key=cv2.contourArea)
        
        # Get bounding rectangle
        x, y, w, h = cv2.boundingRect(largest_contour)
        center_x = x + w // 2
        center_y = y + h // 2
        
        # Check if goal meets size requirements
        if (w < self.goal_detection_params["min_goal_width"] or 
            h < self.goal_detection_params["min_goal_height"]):
            return {
                'detected': False,
                'center_x': 0,
                'center_y': 0,
                'width': 0,
                'height': 0,
                'area': 0
            }
        
        # Check aspect ratio
        aspect_ratio = w / h if h > 0 else 0
        if (aspect_ratio < goal_config["aspect_ratio_min"] or 
            aspect_ratio > goal_config["aspect_ratio_max"]):
            return {
                'detected': False,
                'center_x': 0,
                'center_y': 0,
                'width': 0,
                'height': 0,
                'area': 0
            }
        
        return {
            'detected': True,
            'center_x': center_x,
            'center_y': center_y,
            'width': w,
            'height': h,
            'area': cv2.contourArea(largest_contour)
        }
    
    def get_goal_positions(self):
        """Get current goal positions and detection status"""
        return {
            'blue_goal': {
                'detected': self.blue_goal_detected,
                'center_x': self.blue_goal_center_x,
                'center_y': self.blue_goal_center_y,
                'width': self.blue_goal_width,
                'height': self.blue_goal_height
            },
            'yellow_goal': {
                'detected': self.yellow_goal_detected,
                'center_x': self.yellow_goal_center_x,
                'center_y': self.yellow_goal_center_y,
                'width': self.yellow_goal_width,
                'height': self.yellow_goal_height
            }
        }
    
    def get_goal_navigation_info(self):
        """
        Get goal navigation information for control system
        
        Returns:
            dict: Navigation information for both goals
        """
        blue_info = self._get_goal_navigation_info("blue")
        yellow_info = self._get_goal_navigation_info("yellow")
        
        return {
            'blue_goal': blue_info,
            'yellow_goal': yellow_info
        }
    
    def _get_goal_navigation_info(self, goal_type):
        """
        Get navigation information for a specific goal
        
        Args:
            goal_type: "blue" or "yellow"
            
        Returns:
            dict: Navigation information for the goal
        """
        if goal_type == "blue":
            detected = self.blue_goal_detected
            center_x = self.blue_goal_center_x
            center_y = self.blue_goal_center_y
            width = self.blue_goal_width
            height = self.blue_goal_height
        else:
            detected = self.yellow_goal_detected
            center_x = self.yellow_goal_center_x
            center_y = self.yellow_goal_center_y
            width = self.yellow_goal_width
            height = self.yellow_goal_height
        
        if not detected:
            return {
                'detected': False,
                'horizontal_error': 0,
                'vertical_error': 0,
                'is_centered_horizontally': False,
                'size': 0
            }
        
        # Calculate errors relative to frame center
        horizontal_error = (center_x - self.frame_center_x) / self.frame_center_x
        vertical_error = (center_y - self.frame_center_y) / self.frame_center_y
        
        # Check if goal is centered horizontally
        is_centered_horizontally = abs(horizontal_error) <= self.goal_detection_params["goal_center_tolerance"]
        
        # Calculate goal size (area)
        size = width * height
        
        return {
            'detected': True,
            'horizontal_error': horizontal_error,
            'vertical_error': vertical_error,
            'is_centered_horizontally': is_centered_horizontally,
            'size': size,
            'center_x': center_x,
            'center_y': center_y,
            'width': width,
            'height': height
        }
    
    def stop(self):
        """Stop the camera"""
        self.picam2.stop()
