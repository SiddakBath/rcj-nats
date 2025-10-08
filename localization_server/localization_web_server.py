"""
Localization Web Server for Soccer Robot
Provides real-time visualization of robot position without sensor dependencies
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
    from config import LOCALIZATION_CONFIG
    HAS_HARDWARE = True
except ImportError as e:
    print(f"Warning: Could not import config modules: {e}")
    HAS_HARDWARE = False


class LocalizationWebServer:
    """Web server for real-time localization visualization without sensor dependencies"""
    
    def __init__(self):
        # Simple position tracking without sensors
        self.position = [0, 0]  # x, y coordinates in mm
        self.angle = 0.0  # angle in radians
        self.confidence = 1.0  # confidence level (0-1)
        
        # Debug mode flag
        self.debug_mode = False
        self.debug_lock = threading.Lock()
        
        # Localization data
        self.localization_data = {
            'position': self.position,
            'angle': self.angle,
            'confidence': self.confidence,
            'field_bounds': {
                'width': LOCALIZATION_CONFIG.get('field_width', 3000) if HAS_HARDWARE else 3000,
                'height': LOCALIZATION_CONFIG.get('field_height', 2000) if HAS_HARDWARE else 2000
            },
            'walls': LOCALIZATION_CONFIG.get('walls', []) if HAS_HARDWARE else [],
            'timestamp': time.time()
        }
        
        # Data lock for thread safety
        self.data_lock = threading.Lock()
        
        # Flask app
        self.app = Flask(__name__, template_folder='templates')
        
        # Disable Flask access logs
        log = logging.getLogger('werkzeug')
        log.setLevel(logging.ERROR)
        
        self._setup_routes()
        
        # Setup signal handlers for graceful shutdown
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
    
    def update_position(self, x, y, angle=None):
        """Update robot position manually"""
        with self.data_lock:
            self.position = [x, y]
            if angle is not None:
                self.angle = angle
            self.localization_data.update({
                'position': self.position,
                'angle': self.angle,
                'confidence': self.confidence,
                'timestamp': time.time()
            })
    
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
        
        @self.app.route('/api/position_data')
        def api_position_data():
            """API endpoint to get current position data"""
            with self.data_lock:
                return jsonify({
                    'position': self.localization_data['position'],
                    'angle': self.localization_data['angle'],
                    'confidence': self.localization_data['confidence'],
                    'timestamp': self.localization_data['timestamp']
                })
        
        @self.app.route('/api/field_info')
        def api_field_info():
            """API endpoint to get field configuration"""
            return jsonify({
                'field_bounds': self.localization_data['field_bounds'],
                'walls': self.localization_data['walls'],
                'hardware_available': HAS_HARDWARE
            })
        
        @self.app.route('/api/reset_position', methods=['POST'])
        def api_reset_position():
            """API endpoint to reset robot position"""
            try:
                data = request.get_json() or {}
                x = data.get('x', 0)
                y = data.get('y', 0)
                angle = data.get('angle', 0)
                
                self.update_position(x, y, angle)
                return jsonify({'status': 'success', 'message': f'Position reset to ({x}, {y})'})
                    
            except Exception as e:
                return jsonify({'status': 'error', 'message': str(e)}), 400
        
        @self.app.route('/api/status')
        def api_status():
            """API endpoint to get system status"""
            return jsonify({
                'hardware_available': HAS_HARDWARE,
                'position': self.localization_data['position'],
                'angle': self.localization_data['angle'],
                'confidence': self.localization_data['confidence'],
                'timestamp': time.time()
            })
        
        @self.app.route('/api/update_position', methods=['POST'])
        def api_update_position():
            """API endpoint to update robot position"""
            try:
                data = request.get_json() or {}
                x = data.get('x', self.position[0])
                y = data.get('y', self.position[1])
                angle = data.get('angle', self.angle)
                
                self.update_position(x, y, angle)
                return jsonify({'status': 'success', 'message': f'Position updated to ({x}, {y})'})
                    
            except Exception as e:
                return jsonify({'status': 'error', 'message': str(e)}), 400
        
        @self.app.route('/api/set_debug_mode', methods=['POST'])
        def api_set_debug_mode():
            """API endpoint to toggle debug mode"""
            try:
                data = request.get_json() or {}
                enabled = data.get('enabled', False)
                
                with self.debug_lock:
                    self.debug_mode = enabled
                
                return jsonify({
                    'status': 'success',
                    'debug_mode': self.debug_mode,
                    'message': f"Debug mode {'enabled' if self.debug_mode else 'disabled'}"
                })
            except Exception as e:
                return jsonify({'status': 'error', 'message': str(e)}), 400
        
        @self.app.route('/api/get_debug_mode')
        def api_get_debug_mode():
            """API endpoint to get current debug mode status"""
            with self.debug_lock:
                return jsonify({'debug_mode': self.debug_mode})
    
    def _signal_handler(self, signum, frame):
        """Handle shutdown signals gracefully"""
        print(f"\nReceived signal {signum}, shutting down gracefully...")
        self.cleanup()
        sys.exit(0)
    
    def cleanup(self):
        """Clean up resources"""
        try:
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
