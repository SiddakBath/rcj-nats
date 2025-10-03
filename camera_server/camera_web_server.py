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
        self.upper_orange = np.array([14, 255, 255])
        
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
                
                # Process frame for ball detection
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
        
        # Apply circular mask to ignore center area
        mask = self._apply_circular_mask(mask)
        
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
        
        # Add center crosshair
        height, width = display_frame.shape[:2]
        center_x, center_y = width // 2, height // 2
        cv2.line(display_frame, (center_x - 20, center_y), (center_x + 20, center_y), (255, 255, 255), 1)
        cv2.line(display_frame, (center_x, center_y - 20), (center_x, center_y + 20), (255, 255, 255), 1)
        
        # Draw circular mask indicator
        cv2.circle(display_frame, (self.mask_center_x, self.mask_center_y), self.mask_radius, (0, 0, 255), 2)
        cv2.putText(display_frame, f"Mask R: {self.mask_radius}", 
                   (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        
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
            """API endpoint to get camera and ball detection status"""
            return jsonify({
                'camera_status': 'online',
                'ball_detection': self.last_ball_data,
                'timestamp': time.time()
            })
        
        @self.app.route('/api/ball_data')
        def api_ball_data():
            """API endpoint to get current ball detection data"""
            return jsonify(self.last_ball_data)
        
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

