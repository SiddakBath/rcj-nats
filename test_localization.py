#!/usr/bin/env python3
"""
Test script for the improved localization algorithm
This script tests the grid-based localization without requiring hardware
"""

import sys
import os
import math
import time

# Add the soccer module to the path
sys.path.append(os.path.join(os.path.dirname(__file__), 'soccer'))

from config import LOCALIZATION_CONFIG, TOF_CONFIG
from localization import Localizer

class MockTOFManager:
    """Mock TOF manager for testing without hardware"""
    
    def __init__(self):
        self.sensors = []
        self.sensor_distances = {}
        
        # Create mock sensors based on config
        for i, address in enumerate(TOF_CONFIG["addresses"]):
            angle_degrees = TOF_CONFIG["angles"][i]
            angle_radians = math.radians(angle_degrees)
            
            # Create mock sensor
            sensor = MockTOFSensor(address, angle_radians)
            self.sensors.append(sensor)
            self.sensor_distances[angle_radians] = 0
    
    def update_distances(self):
        """Update all sensor distance readings with mock data"""
        for sensor in self.sensors:
            self.sensor_distances[sensor.angle] = sensor.get_distance()
    
    def get_all_distances(self):
        """Get all distance measurements"""
        return self.sensor_distances.copy()
    
    def get_sensor_count(self):
        """Get number of active sensors"""
        return len(self.sensors)

class MockTOFSensor:
    """Mock TOF sensor for testing"""
    
    def __init__(self, address, angle):
        self.address = address
        self.angle = angle
        self.distance = 1000  # Default distance
    
    def get_distance(self):
        """Get mock distance reading"""
        return self.distance

class MockIMUSensor:
    """Mock IMU sensor for testing"""
    
    def __init__(self):
        self.heading = 0.0
        self.initial_heading = None
    
    def get_compass_heading(self):
        """Get mock compass heading"""
        return self.heading
    
    def get_relative_heading(self):
        """Get mock relative heading"""
        if self.initial_heading is None:
            self.initial_heading = self.heading
        relative = self.heading - self.initial_heading
        if relative < 0:
            relative += 360
        elif relative >= 360:
            relative -= 360
        return relative

def test_localization_algorithm():
    """Test the localization algorithm with mock data"""
    print("Testing improved localization algorithm...")
    
    # Create mock hardware
    mock_tof_manager = MockTOFManager()
    mock_imu_sensor = MockIMUSensor()
    
    # Create localizer with mock hardware
    localizer = Localizer(
        i2c_bus=None,
        tof_manager=mock_tof_manager,
        imu_sensor=mock_imu_sensor
    )
    
    print(f"Field dimensions: {localizer.field_width} x {localizer.field_height} mm")
    print(f"Grid resolution: {localizer.grid_resolution} mm")
    print(f"Angle resolution: {localizer.angle_resolution} bins")
    
    # Test different robot positions
    test_positions = [
        [1215, 910],   # Center of field
        [600, 400],    # Near corner
        [1800, 1400],  # Near opposite corner
        [1215, 200],   # Near edge
    ]
    
    for i, test_pos in enumerate(test_positions):
        print(f"\n--- Test {i+1}: Robot at {test_pos} mm ---")
        
        # Set mock sensor readings for this position
        for sensor in mock_tof_manager.sensors:
            # Simulate distance readings based on position
            world_angle = sensor.angle
            # Simple distance calculation (not accurate, just for testing)
            distance = 500 + (test_pos[0] + test_pos[1]) % 1000
            sensor.distance = distance
        
        # Update sensor readings
        mock_tof_manager.update_distances()
        
        # Set robot angle
        mock_imu_sensor.heading = 0.0
        
        # Perform localization
        start_time = time.time()
        position, confidence = localizer.localize()
        localization_time = time.time() - start_time
        
        print(f"Estimated position: ({position[0]:.1f}, {position[1]:.1f}) mm")
        print(f"Confidence: {confidence:.3f}")
        print(f"Localization time: {localization_time*1000:.1f}ms")
        
        # Calculate error
        error = math.sqrt((position[0] - test_pos[0])**2 + (position[1] - test_pos[1])**2)
        print(f"Position error: {error:.1f} mm")
        
        # Validate result
        is_valid = localizer.validate_localization(position, localizer.angle)
        print(f"Validation: {'PASS' if is_valid else 'FAIL'}")
    
    print("\n--- Testing lookup table initialization ---")
    if not localizer.lookup_table_initialized:
        print("Initializing lookup table...")
        localizer._initialize_lookup_table()
        print(f"Lookup table initialized with {len(localizer.lookup_table)} entries")
    else:
        print("Lookup table already initialized")
    
    print("\n--- Testing global grid search ---")
    position, error = localizer._global_grid_search(0.0)
    print(f"Global search result: ({position[0]:.1f}, {position[1]:.1f}) mm, error: {error:.1f}")
    
    print("\nLocalization algorithm test completed!")

if __name__ == "__main__":
    test_localization_algorithm()
