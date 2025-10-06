"""
Localization Web Server for Soccer Robot
Provides real-time visualization of robot position and sensor data
"""

import sys
import os
import time
import threading
import signal
import json
import math
import logging
from flask import Flask, render_template, Response, jsonify, request
import numpy as np

# Add the soccer module to the path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'soccer'))

try:
    from localization import Localizer
    from config import LOCALIZATION_CONFIG, TOF_CONFIG
    from tof_sensor import TOFManager
    from imu_sensor import IMUSensor
    import board
    import busio
    HAS_HARDWARE = True
except ImportError as e:
    print(f"Warning: Could not import hardware modules: {e}")
    HAS_HARDWARE = False


class LocalizationWebServer:
    """Web server for real-time localization visualization"""
    
    def __init__(self):
        # Initialize hardware if available
        self.localizer = None
        self.i2c = None
        self.hardware_available = False
        
        if HAS_HARDWARE:
            try:
                # Initialize I2C bus
                self.i2c = busio.I2C(board.SCL, board.SDA)
                
                # Initialize localization system
                self.localizer = Localizer(i2c_bus=self.i2c)
                
                if self.localizer.tof_manager.get_sensor_count() > 0:
                    self.hardware_available = True
                    print(f"Hardware initialized with {self.localizer.tof_manager.get_sensor_count()} TOF sensors")
                else:
                    print("No TOF sensors detected")
            except Exception as e:
                print(f"Hardware initialization failed: {e}")
                self.hardware_available = False
        
        # Localization data
        self.localization_data = {
            'position': [0, 0],
            'angle': 0.0,
            'confidence': 0.0,
            'sensor_data': {},
            'field_bounds': {
                'width': LOCALIZATION_CONFIG.get('field_width', 3000),
                'height': LOCALIZATION_CONFIG.get('field_height', 2000)
            },
            'walls': LOCALIZATION_CONFIG.get('walls', []),
            'timestamp': time.time()
        }
        
        # Data lock for thread safety
        self.data_lock = threading.Lock()
        
        # Start localization thread if hardware is available
        if self.hardware_available:
            self.localization_thread = threading.Thread(target=self._localization_loop)
            self.localization_thread.daemon = True
            self.localization_thread.start()
        
        # Flask app
        self.app = Flask(__name__, template_folder='templates')
        
        # Disable Flask access logs
        log = logging.getLogger('werkzeug')
        log.setLevel(logging.ERROR)
        
        self._setup_routes()
        
        # Setup signal handlers for graceful shutdown
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
    
    def _localization_loop(self):
        """Continuous localization loop"""
        # Sensor logging variables
        last_sensor_log_time = 0
        sensor_log_interval = 2.0  # Log closest sensor every 2 seconds
        
        while True:
            try:
                if self.localizer:
                    # Perform localization
                    position, confidence = self.localizer.localize()
                    angle = self.localizer.get_angle()
                    
                    # Get sensor data
                    sensor_data = self.localizer.get_sensor_data()
                    
                    # Update data with thread safety
                    with self.data_lock:
                        self.localization_data.update({
                            'position': position,
                            'angle': angle,
                            'confidence': confidence,
                            'sensor_data': sensor_data,
                            'timestamp': time.time()
                        })
                    
                    # Log closest sensor every 2 seconds
                    current_time = time.time()
                    if current_time - last_sensor_log_time >= sensor_log_interval:
                        closest_angle, closest_distance, closest_direction = self.localizer.get_closest_sensor()
                        if closest_angle is not None:
                            print(f"[SENSOR LOG] Closest sensor: {closest_direction} "
                                  f"(angle: {math.degrees(closest_angle):.1f}°, "
                                  f"distance: {closest_distance:.1f}mm)")
                        else:
                            print("[SENSOR LOG] No valid sensors detected")
                        last_sensor_log_time = current_time
                
                # Update frequency from config
                time.sleep(LOCALIZATION_CONFIG.get('update_frequency', 0.1))
                
            except Exception as e:
                print(f"Error in localization loop: {e}")
                time.sleep(1.0)
    
    def _setup_routes(self):
        """Setup Flask routes"""
        
        @self.app.route('/')
        def index():
            return render_template('localization_viewer.html')
        
        @self.app.route('/api/localization_data')
        def api_localization_data():
            """API endpoint to get current localization data"""
            with self.data_lock:
                return jsonify(self.localization_data)
        
        @self.app.route('/api/sensor_data')
        def api_sensor_data():
            """API endpoint to get detailed sensor data"""
            with self.data_lock:
                sensor_data = self.localization_data.get('sensor_data', {})
                # Add I2C addresses from config
                if self.localizer and hasattr(self.localizer, 'tof_manager'):
                    sensor_data['i2c_addresses'] = TOF_CONFIG.get('addresses', [])
                return jsonify(sensor_data)
        
        @self.app.route('/api/field_info')
        def api_field_info():
            """API endpoint to get field configuration"""
            return jsonify({
                'field_bounds': self.localization_data['field_bounds'],
                'walls': self.localization_data['walls'],
                'hardware_available': self.hardware_available
            })
        
        @self.app.route('/api/reset_position', methods=['POST'])
        def api_reset_position():
            """API endpoint to reset robot position"""
            try:
                data = request.get_json() or {}
                x = data.get('x', 0)
                y = data.get('y', 0)
                
                if self.localizer:
                    self.localizer.reset_position(x, y)
                    return jsonify({'status': 'success', 'message': f'Position reset to ({x}, {y})'})
                else:
                    return jsonify({'status': 'error', 'message': 'Localizer not available'}), 400
                    
            except Exception as e:
                return jsonify({'status': 'error', 'message': str(e)}), 400
        
        @self.app.route('/api/status')
        def api_status():
            """API endpoint to get system status"""
            return jsonify({
                'hardware_available': self.hardware_available,
                'sensor_count': self.localization_data.get('sensor_data', {}).get('sensor_count', 0),
                'valid_measurements': self.localization_data.get('sensor_data', {}).get('valid_measurements', 0),
                'timestamp': time.time()
            })
    
    def _signal_handler(self, signum, frame):
        """Handle shutdown signals gracefully"""
        print(f"\nReceived signal {signum}, shutting down gracefully...")
        self.cleanup()
        sys.exit(0)
    
    def cleanup(self):
        """Clean up resources"""
        try:
            if self.localizer and hasattr(self.localizer, 'tof_manager'):
                # Clean up TOF sensors if needed
                pass
            print("Localization server cleanup complete")
        except Exception as e:
            print(f"Error during cleanup: {e}")
    
    def run(self, host='0.0.0.0', port=5001, debug=False):
        """Run the web server"""
        print(f"Starting localization web server on http://{host}:{port}")
        print("Open your web browser and navigate to the URL above to view the localization data")
        try:
            self.app.run(host=host, port=port, debug=debug, threaded=True)
        except KeyboardInterrupt:
            print("Shutting down web server...")
        finally:
            self.cleanup()


def main():
    try:
        server = LocalizationWebServer()
        server.run(host='0.0.0.0', port=5001, debug=False)
    except KeyboardInterrupt:
        print("\nShutting down localization server...")
    except Exception as e:
        print(f"Error: {e}")


if __name__ == "__main__":
    main()
