import cv2
import numpy as np
from flask import Flask, render_template, Response, jsonify, request
from picamera2 import Picamera2
import threading
import time
import json
import os
import signal
import sys

class CameraWebServer:
    def __init__(self):
        # Initialize Pi Camera
        self.picam2 = Picamera2()
        self.picam2.configure(self.picam2.create_video_configuration(
            main={"size": (640, 480), "format": "RGB888"}
        ))
        self.picam2.start()
        
        # Ball detection parameters
        self.lower_orange = np.array([0, 132, 61])
        self.upper_orange = np.array([20, 255, 255])
        
        # Goal detection parameters
        # Blue goal (target goal) - using config.py values
        self.blue_goal_lower = np.array([89, 75, 78])
        self.blue_goal_upper = np.array([145, 255, 255])
        
        # Yellow goal (opponent goal) - using config.py values
        self.yellow_goal_lower = np.array([18, 179, 179])
        self.yellow_goal_upper = np.array([65, 255, 255])
        
        # Goal detection thresholds (adjusted for smaller goals)
        self.min_goal_area = 50  # Reduced from 100
        self.max_goal_area = 50000
        self.min_goal_width = 25  # Reduced from 50
        self.min_goal_height = 15  # Reduced from 30
        
        # Circular mask parameters to ignore center area
        self.mask_center_x = 320  # Center of 640x480 frame
        self.mask_center_y = 240
        self.mask_radius = 110  # 20% bigger than 92 (92 * 1.20 = 110)
        
        # Frame processing
        self.frame = None
        self.frame_lock = threading.Lock()
        self.last_ball_data = {
            'detected': False,
            'center': (0, 0),
            'radius': 0,
            'area': 0
        }
        self.last_goal_data = {
            'blue_goal': {
                'detected': False,
                'center': (0, 0),
                'width': 0,
                'height': 0,
                'area': 0
            },
            'yellow_goal': {
                'detected': False,
                'center': (0, 0),
                'width': 0,
                'height': 0,
                'area': 0
            }
        }
        
        # Start frame capture thread
        self.capture_thread = threading.Thread(target=self._capture_frames)
        self.capture_thread.daemon = True
        self.capture_thread.start()
        
        # Flask app
        self.app = Flask(__name__, template_folder='templates')
        self._setup_routes()
        
        # Setup signal handlers for graceful shutdown
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
    
    def _capture_frames(self):
        """Continuously capture frames from the camera"""
        while True:
            try:
                # Capture frame from Pi Camera
                frame = self.picam2.capture_array()
                
                # Convert to HSV for processing
                hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                
                # Process frame for ball and goal detection
                processed_frame = self._process_frame(hsv_frame, frame)
                
                with self.frame_lock:
                    self.frame = processed_frame
                    
            except Exception as e:
                print(f"Error capturing frame: {e}")
                # Add longer sleep on error to prevent rapid retry loops
                time.sleep(1.0)
            else:
                # Normal operation - small delay to prevent excessive CPU usage
                time.sleep(0.033)  # ~30 FPS
    
    def _process_frame(self, hsv_frame, rgb_frame):
        """Process frame for ball detection and add visual indicators"""
        # Create a copy of RGB frame for display
        display_frame = rgb_frame.copy()
        
        # Ball detection using HSV frame
        mask = cv2.inRange(hsv_frame, self.lower_orange, self.upper_orange)
        
        # Apply circular mask to ignore center area (temporarily disabled)
        # mask = self._apply_circular_mask(mask)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        # Filter contours by area - reduced minimum area to detect smaller balls (radius ~7 or smaller)
        filtered_contours = [x for x in contours if cv2.contourArea(x) > 20 and cv2.contourArea(x) < 30000]
        
        if filtered_contours:
            largest_contour = max(filtered_contours, key=cv2.contourArea)
            (x, y), radius = cv2.minEnclosingCircle(largest_contour)
            
            center = (int(x), int(y))
            radius = int(radius)
            area = cv2.contourArea(largest_contour)
            
            # Update ball data for API
            self.last_ball_data = {
                'detected': True,
                'center': center,
                'radius': radius,
                'area': area
            }
            
            # Print ball area for debugging
            print(f"Ball detected - Area: {area:.1f}, Center: ({center[0]}, {center[1]}), Radius: {radius}")
            
            # Draw circle around detected ball
            cv2.circle(display_frame, center, radius, (0, 255, 0), 2)
            cv2.circle(display_frame, center, 2, (0, 255, 0), -1)
            
            # Add text label with area information
            cv2.putText(display_frame, f"Ball: ({center[0]}, {center[1]}) Area: {area:.0f}", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            # Update ball data for API
            self.last_ball_data = {
                'detected': False,
                'center': (0, 0),
                'radius': 0,
                'area': 0
            }
            
            cv2.putText(display_frame, "No ball detected", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
        
        # Detect goals in the frame
        blue_goal, yellow_goal = self._detect_goals(hsv_frame, display_frame)
        
        # Add center crosshair
        height, width = display_frame.shape[:2]
        center_x, center_y = width // 2, height // 2
        cv2.line(display_frame, (center_x - 20, center_y), (center_x + 20, center_y), (255, 255, 255), 1)
        cv2.line(display_frame, (center_x, center_y - 20), (center_x, center_y + 20), (255, 255, 255), 1)
        
        # Draw circular mask indicator
        cv2.circle(display_frame, (self.mask_center_x, self.mask_center_y), self.mask_radius, (0, 0, 255), 2)
        cv2.putText(display_frame, f"Mask R: {self.mask_radius}", 
                   (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        
        # Add goal detection status text
        goal_status_y = 90
        if blue_goal['detected']:
            cv2.putText(display_frame, f"Blue Goal: ({blue_goal['center'][0]}, {blue_goal['center'][1]})", 
                       (10, goal_status_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
            goal_status_y += 20
        else:
            cv2.putText(display_frame, "Blue Goal: Not detected", 
                       (10, goal_status_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (128, 128, 128), 1)
            goal_status_y += 20
        
        if yellow_goal['detected']:
            cv2.putText(display_frame, f"Yellow Goal: ({yellow_goal['center'][0]}, {yellow_goal['center'][1]})", 
                       (10, goal_status_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        else:
            cv2.putText(display_frame, "Yellow Goal: Not detected", 
                       (10, goal_status_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (128, 128, 128), 1)
        
        return display_frame
    
    def _apply_circular_mask(self, mask):
        """Apply a circular mask to ignore the center area of the frame"""
        # Create a circular mask (white circle on black background)
        mask_with_circle = np.zeros_like(mask)
        cv2.circle(mask_with_circle, (self.mask_center_x, self.mask_center_y), self.mask_radius, 255, -1)
        
        # Invert the circle mask (black circle on white background)
        circle_mask = cv2.bitwise_not(mask_with_circle)
        
        # Apply the mask to the original detection mask
        masked_result = cv2.bitwise_and(mask, circle_mask)
        
        return masked_result
    
    def _detect_goals(self, hsv_frame, display_frame):
        """Detect blue and yellow goals in the frame"""
        # Detect blue goal (target goal)
        blue_goal_result = self._detect_single_goal(hsv_frame, self.blue_goal_lower, self.blue_goal_upper, "blue")
        self.last_goal_data['blue_goal'] = blue_goal_result
        
        # Detect yellow goal (opponent goal)
        yellow_goal_result = self._detect_single_goal(hsv_frame, self.yellow_goal_lower, self.yellow_goal_upper, "yellow")
        self.last_goal_data['yellow_goal'] = yellow_goal_result
        
        # Draw goal visualizations
        self._draw_goal_visualizations(display_frame, blue_goal_result, yellow_goal_result)
        
        return blue_goal_result, yellow_goal_result
    
    def _detect_single_goal(self, hsv_frame, lower_color, upper_color, goal_type):
        """Detect a single goal using HSV color filtering"""
        # Create HSV mask for the goal color
        mask = cv2.inRange(hsv_frame, lower_color, upper_color)
        
        # Apply circular mask to ignore center area (temporarily disabled)
        # mask = self._apply_circular_mask(mask)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        # Filter contours by area
        filtered_contours = [c for c in contours if 
                           cv2.contourArea(c) > self.min_goal_area and 
                           cv2.contourArea(c) < self.max_goal_area]
        
        if not filtered_contours:
            return {
                'detected': False,
                'center': (0, 0),
                'width': 0,
                'height': 0,
                'area': 0
            }
        
        # Find the largest contour
        largest_contour = max(filtered_contours, key=cv2.contourArea)
        
        # Get bounding rectangle
        x, y, w, h = cv2.boundingRect(largest_contour)
        center = (x + w // 2, y + h // 2)
        
        # Check if goal meets size requirements
        if w < self.min_goal_width or h < self.min_goal_height:
            return {
                'detected': False,
                'center': (0, 0),
                'width': 0,
                'height': 0,
                'area': 0
            }
        
        return {
            'detected': True,
            'center': center,
            'width': w,
            'height': h,
            'area': cv2.contourArea(largest_contour)
        }
    
    def _draw_goal_visualizations(self, display_frame, blue_goal, yellow_goal):
        """Draw goal visualizations including lines from center to goals"""
        height, width = display_frame.shape[:2]
        center_x, center_y = width // 2, height // 2
        
        # Draw blue goal (target goal)
        if blue_goal['detected']:
            center = blue_goal['center']
            # Draw rectangle around goal
            x, y = center[0] - blue_goal['width']//2, center[1] - blue_goal['height']//2
            cv2.rectangle(display_frame, (x, y), (x + blue_goal['width'], y + blue_goal['height']), (255, 0, 0), 2)
            # Draw line from center to goal
            cv2.line(display_frame, (center_x, center_y), center, (255, 0, 0), 3)
            # Add text label
            cv2.putText(display_frame, f"BLUE GOAL: ({center[0]}, {center[1]})", 
                       (center[0] + 10, center[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
        
        # Draw yellow goal (opponent goal)
        if yellow_goal['detected']:
            center = yellow_goal['center']
            # Draw rectangle around goal
            x, y = center[0] - yellow_goal['width']//2, center[1] - yellow_goal['height']//2
            cv2.rectangle(display_frame, (x, y), (x + yellow_goal['width'], y + yellow_goal['height']), (0, 255, 255), 2)
            # Draw line from center to goal
            cv2.line(display_frame, (center_x, center_y), center, (0, 255, 255), 3)
            # Add text label
            cv2.putText(display_frame, f"YELLOW GOAL: ({center[0]}, {center[1]})", 
                       (center[0] + 10, center[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    
    def _generate_frames(self):
        """Generate frames for streaming"""
        while True:
            with self.frame_lock:
                if self.frame is not None:
                    # Encode frame as JPEG
                    ret, buffer = cv2.imencode('.jpg', self.frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                    if ret:
                        frame_bytes = buffer.tobytes()
                        yield (b'--frame\r\n'
                               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
            time.sleep(0.033)  # ~30 FPS
    
    def _setup_routes(self):
        """Setup Flask routes"""
        
        @self.app.route('/')
        def index():
            return render_template('camera_viewer.html')
        
        @self.app.route('/video_feed')
        def video_feed():
            return Response(self._generate_frames(),
                          mimetype='multipart/x-mixed-replace; boundary=frame')
        
        @self.app.route('/api/status')
        def api_status():
            """API endpoint to get camera, ball detection, and goal detection status"""
            return jsonify({
                'camera_status': 'online',
                'ball_detection': self.last_ball_data,
                'goal_detection': self.last_goal_data,
                'timestamp': time.time()
            })
        
        @self.app.route('/api/ball_data')
        def api_ball_data():
            """API endpoint to get current ball detection data"""
            return jsonify(self.last_ball_data)
        
        @self.app.route('/api/goal_data')
        def api_goal_data():
            """API endpoint to get current goal detection data"""
            return jsonify(self.last_goal_data)
        
        @self.app.route('/api/stop_robot', methods=['POST'])
        def api_stop_robot():
            """API endpoint to stop the robot"""
            # This would integrate with the robot control system
            return jsonify({'status': 'success', 'message': 'Stop command sent to robot'})
        
        @self.app.route('/api/update_detection_params', methods=['POST'])
        def api_update_detection_params():
            """API endpoint to update ball detection parameters"""
            try:
                data = request.get_json()
                if 'lower_orange' in data:
                    self.lower_orange = np.array(data['lower_orange'])
                if 'upper_orange' in data:
                    self.upper_orange = np.array(data['upper_orange'])
                if 'mask_radius' in data:
                    self.mask_radius = int(data['mask_radius'])
                if 'mask_center_x' in data:
                    self.mask_center_x = int(data['mask_center_x'])
                if 'mask_center_y' in data:
                    self.mask_center_y = int(data['mask_center_y'])
                return jsonify({'status': 'success', 'message': 'Detection parameters updated'})
            except Exception as e:
                return jsonify({'status': 'error', 'message': str(e)}), 400
        
        @self.app.route('/api/update_goal_colors', methods=['POST'])
        def api_update_goal_colors():
            """API endpoint to update goal color detection parameters"""
            try:
                data = request.get_json()
                
                # Update blue goal colors
                if 'blue_goal_lower' in data:
                    self.blue_goal_lower = np.array(data['blue_goal_lower'])
                if 'blue_goal_upper' in data:
                    self.blue_goal_upper = np.array(data['blue_goal_upper'])
                
                # Update yellow goal colors
                if 'yellow_goal_lower' in data:
                    self.yellow_goal_lower = np.array(data['yellow_goal_lower'])
                if 'yellow_goal_upper' in data:
                    self.yellow_goal_upper = np.array(data['yellow_goal_upper'])
                
                return jsonify({'status': 'success', 'message': 'Goal color parameters updated'})
            except Exception as e:
                return jsonify({'status': 'error', 'message': str(e)}), 400
        
        @self.app.route('/api/get_goal_colors')
        def api_get_goal_colors():
            """API endpoint to get current goal color parameters"""
            return jsonify({
                'blue_goal_lower': self.blue_goal_lower.tolist(),
                'blue_goal_upper': self.blue_goal_upper.tolist(),
                'yellow_goal_lower': self.yellow_goal_lower.tolist(),
                'yellow_goal_upper': self.yellow_goal_upper.tolist()
            })
    
    def _signal_handler(self, signum, frame):
        """Handle shutdown signals gracefully"""
        print(f"\nReceived signal {signum}, shutting down gracefully...")
        self.cleanup()
        sys.exit(0)
    
    def cleanup(self):
        """Clean up camera resources"""
        try:
            if hasattr(self, 'picam2'):
                self.picam2.stop()
                print("Camera stopped successfully")
        except Exception as e:
            print(f"Error stopping camera: {e}")
    
    def run(self, host='0.0.0.0', port=5000, debug=False):
        """Run the web server"""
        print(f"Starting camera web server on http://{host}:{port}")
        print("Open your web browser and navigate to the URL above to view the camera feed")
        try:
            self.app.run(host=host, port=port, debug=debug, threaded=True)
        except KeyboardInterrupt:
            print("Shutting down web server...")
        finally:
            self.cleanup()

def main():
    try:
        server = CameraWebServer()
        server.run(host='0.0.0.0', port=5000, debug=False)
    except KeyboardInterrupt:
        print("\nShutting down web server...")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()

