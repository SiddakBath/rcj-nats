"""
IMU Sensor module for the Soccer Robot
Handles BNO085 IMU sensor for compass and orientation data
"""

import math
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C
from config import IMU_CONFIG


class IMUSensor:
    """Handles BNO085 IMU sensor for compass and orientation data"""
    
    def __init__(self, i2c_bus):
        """
        Initialize IMU sensor
        
        Args:
            i2c_bus: I2C bus instance for IMU communication
        """
        self.i2c = i2c_bus
        self.bno = None
        self.initial_heading = None
        self.current_heading = 0.0
        self.is_initialized = False
        
        # Initialize IMU
        self.setup_imu()
    
    def setup_imu(self):
        """Initialize the BNO085 IMU sensor for compass functionality"""
        try:
            # Initialize BNO085 using the standard I2C approach
            self.bno = BNO08X_I2C(self.i2c)
            
            # Enable the reports from config
            for report in IMU_CONFIG["enable_reports"]:
                if report == "BNO_REPORT_ACCELEROMETER":
                    self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
                elif report == "BNO_REPORT_GYROSCOPE":
                    self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
                elif report == "BNO_REPORT_MAGNETOMETER":
                    self.bno.enable_feature(BNO_REPORT_MAGNETOMETER)
                elif report == "BNO_REPORT_ROTATION_VECTOR":
                    self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
            
            self.is_initialized = True
            print("BNO085 IMU initialized successfully")
            
        except Exception as e:
            print(f"Failed to initialize BNO085 IMU: {e}")
            self.bno = None
            self.is_initialized = False
    
    def get_compass_heading(self):
        """
        Get the current compass heading in degrees (0-360)
        
        Returns:
            float: Compass heading in degrees (0-360) or None if error
        """
        if self.bno is None:
            return None
        
        try:
            # Get rotation vector (quaternion)
            quat_i, quat_j, quat_k, quat_real = self.bno.quaternion
            
            # Convert quaternion to Euler angles
            # Calculate yaw (heading) from quaternion
            yaw = math.atan2(2.0 * (quat_real * quat_k + quat_i * quat_j),
                           1.0 - 2.0 * (quat_j * quat_j + quat_k * quat_k))
            
            # Convert from radians to degrees and normalize to 0-360
            heading = math.degrees(yaw)
            if heading < 0:
                heading += 360
            
            # Set initial heading on first reading
            if self.initial_heading is None:
                self.initial_heading = heading
            
            self.current_heading = heading
            return heading
            
        except Exception as e:
            print(f"Error reading compass heading: {e}")
            return None
    
    def get_relative_heading(self):
        """
        Get heading relative to initial orientation (0 = initial direction)
        
        Returns:
            float: Relative heading in degrees (0-360) or None if error
        """
        if not self.is_available():
            return None
        
        current = self.get_compass_heading()
        if current is None:
            return None
        
        # If no initial heading set, set it to current heading
        if self.initial_heading is None:
            self.initial_heading = current
            print(f"Initial heading set to: {current:.1f}°")
        
        # Calculate relative heading
        relative = current - self.initial_heading
        if relative < 0:
            relative += 360
        elif relative >= 360:
            relative -= 360
        
        return relative
    
    def get_imu_data(self):
        """
        Get comprehensive IMU data including accelerometer, gyroscope, and magnetometer
        
        Returns:
            dict: Complete IMU data or None if error
        """
        if self.bno is None:
            return None
        
        try:
            data = {
                'heading': self.get_compass_heading(),
                'relative_heading': self.get_relative_heading(),
                'acceleration': self.bno.acceleration,
                'gyro': self.bno.gyro,
                'magnetic': self.bno.magnetic
            }
            return data
        except Exception as e:
            print(f"Error reading IMU data: {e}")
            return None
    
    def get_orientation_info(self):
        """
        Get formatted orientation information for display
        
        Returns:
            dict: Formatted orientation data
        """
        heading = self.get_compass_heading()
        relative_heading = self.get_relative_heading()
        
        return {
            'heading': heading,
            'relative_heading': relative_heading,
            'heading_str': f"Heading: {heading:.1f}°" if heading is not None else "Heading: N/A",
            'relative_str': f"Rel: {relative_heading:.1f}°" if relative_heading is not None else "Rel: N/A"
        }
    
    def is_available(self):
        """
        Check if IMU is available and working
        
        Returns:
            bool: True if IMU is available, False otherwise
        """
        return self.is_initialized and self.bno is not None
    
    def reset_initial_heading(self):
        """Reset the initial heading to current heading"""
        current = self.get_compass_heading()
        if current is not None:
            self.initial_heading = current
            print(f"Initial heading reset to: {current:.1f}°")
        else:
            print("Cannot reset initial heading - IMU not available")
    
    def initialize_relative_heading(self):
        """
        Force initialization of relative heading to current compass heading
        This should be called when the robot starts or repositions
        
        Returns:
            bool: True if initialization successful, False otherwise
        """
        if not self.is_available():
            print("Cannot initialize relative heading - IMU not available")
            return False
        
        current = self.get_compass_heading()
        if current is not None:
            self.initial_heading = current
            print(f"Relative heading initialized to: {current:.1f}°")
            return True
        else:
            print("Cannot initialize relative heading - no compass reading available")
            return False
