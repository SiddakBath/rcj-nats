#!/usr/bin/env python3
"""
Test script for the enhanced localization system with triangulation
"""

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), 'soccer'))

from soccer.localization import Localizer
from soccer.config import LOCALIZATION_CONFIG, TOF_CONFIG
import math

def test_localization_system():
    """Test the localization system with triangulation"""
    print("=" * 60)
    print("SOCCER ROBOT LOCALIZATION SYSTEM TEST")
    print("=" * 60)
    
    try:
        # Initialize localization system
        print("Initializing localization system...")
        localizer = Localizer()
        
        # Test system status
        print("\n1. SYSTEM STATUS")
        print("-" * 30)
        
        # Get sensor health
        sensor_health = localizer.get_sensor_health_status()
        print(f"Sensor Health: {sensor_health['health_percentage']:.1f}% ({sensor_health['healthy_sensors']}/{sensor_health['total_sensors']} sensors)")
        
        # Get IMU status
        imu_status = localizer.get_imu_status()
        print(f"IMU Status: {'Available' if imu_status['available'] else 'Not Available'}")
        if imu_status['available']:
            print(f"  Compass Heading: {imu_status.get('compass_heading_deg', 'N/A'):.1f}°")
            print(f"  Relative Heading: {imu_status.get('relative_heading_deg', 'N/A'):.1f}°")
        
        # Test field configuration
        print("\n2. FIELD CONFIGURATION")
        print("-" * 30)
        print(f"Field Dimensions: {LOCALIZATION_CONFIG['field_width']} x {LOCALIZATION_CONFIG['field_height']} mm")
        print(f"Number of Walls: {len(LOCALIZATION_CONFIG['walls'])}")
        print(f"TOF Sensors: {len(TOF_CONFIG['addresses'])}")
        
        # Display sensor configuration
        print("\n3. SENSOR CONFIGURATION")
        print("-" * 30)
        for i, (addr, angle) in enumerate(zip(TOF_CONFIG['addresses'], TOF_CONFIG['angles'])):
            print(f"Sensor {i+1}: Address 0x{addr:02x}, Angle {angle:6.1f}°")
        
        # Test ray casting
        print("\n4. RAY CASTING TEST")
        print("-" * 30)
        test_positions = [
            [1215, 910],   # Center
            [500, 500],    # Near corner
            [2000, 1500],  # Near opposite corner
        ]
        
        for pos in test_positions:
            print(f"\nTesting position ({pos[0]}, {pos[1]}):")
            for sensor in localizer.tof_manager.sensors:
                distance = localizer._ray_cast_to_walls(pos[0], pos[1], sensor.angle)
                if distance is not None:
                    print(f"  Sensor {math.degrees(sensor.angle):6.1f}°: {distance:6.1f} mm")
                else:
                    print(f"  Sensor {math.degrees(sensor.angle):6.1f}°: No intersection")
        
        # Test localization accuracy
        print("\n5. LOCALIZATION ACCURACY")
        print("-" * 30)
        accuracy = localizer.get_localization_accuracy()
        print(f"Accuracy Level: {accuracy['accuracy']}")
        print(f"Error Estimate: {accuracy.get('error_estimate', 'N/A')}")
        print(f"Valid Sensors: {accuracy.get('valid_sensor_count', 0)}")
        
        # Test bounds checking
        print("\n6. FIELD BOUNDS CHECK")
        print("-" * 30)
        bounds = localizer.get_field_bounds_check()
        print(f"Within Bounds: {bounds['within_bounds']}")
        print(f"Current Position: ({bounds['position'][0]:.1f}, {bounds['position'][1]:.1f})")
        print(f"Distance to Edges:")
        for edge, distance in bounds['distance_to_edges'].items():
            print(f"  {edge.capitalize()}: {distance:.1f} mm")
        
        # Test triangulation system
        print("\n7. TRIANGULATION SYSTEM TEST")
        print("-" * 30)
        test_results = localizer.test_localization_system()
        print(f"System Status: {test_results['system_status']}")
        
        # Display sensor readings
        print("\n8. CURRENT SENSOR READINGS")
        print("-" * 30)
        sensor_data = localizer.get_sensor_data()
        print(f"Robot Position: ({sensor_data['position'][0]:.1f}, {sensor_data['position'][1]:.1f})")
        print(f"Robot Angle: {math.degrees(sensor_data['robot_angle']):.1f}°")
        print(f"TOF Distances:")
        for angle, distance in sensor_data['tof_distances'].items():
            print(f"  {math.degrees(angle):6.1f}°: {distance:4d} mm")
        
        print("\n" + "=" * 60)
        print("LOCALIZATION SYSTEM TEST COMPLETED")
        print("=" * 60)
        
        return True
        
    except Exception as e:
        print(f"Error during localization test: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    success = test_localization_system()
    sys.exit(0 if success else 1)
