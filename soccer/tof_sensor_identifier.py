#!/usr/bin/env python3
"""
TOF Sensor Identifier Script
Continuously reads TOF sensors and shows which ones are closest to walls.
This helps identify which sensor is positioned where on the robot.
"""

import time
import math
import sys

from tof_sensor import TOFManager
from config import TOF_CONFIG


class TOFIdentifier:
    """TOF sensor identifier for mapping sensor positions"""
    
    def __init__(self):
        """Initialize the TOF identifier"""
        self.tof_manager = None
        self.running = False
        
    def initialize_sensors(self):
        """Initialize TOF sensors"""
        try:
            print("Initializing TOF sensors...")
            self.tof_manager = TOFManager()
            
            if self.tof_manager.get_sensor_count() == 0:
                print("❌ No TOF sensors found!")
                return False
                
            print(f"✅ Found {self.tof_manager.get_sensor_count()} TOF sensors")
            return True
            
        except Exception as e:
            print(f"❌ Failed to initialize TOF sensors: {e}")
            return False
    
    def get_sensor_ranking(self):
        """
        Get sensors ranked by distance (closest first)
        
        Returns:
            list: List of (sensor, distance, angle_deg) tuples sorted by distance
        """
        # Update all sensor readings
        self.tof_manager.update_distances()
        
        # Get all sensor info with distances
        sensor_data = []
        for sensor in self.tof_manager.sensors:
            distance = self.tof_manager.sensor_distances[sensor.angle]
            angle_deg = sensor.get_angle_degrees()
            sensor_data.append((sensor, distance, angle_deg))
        
        # Sort by distance (closest first)
        sensor_data.sort(key=lambda x: x[1])
        
        return sensor_data
    
    def display_sensor_info(self, sensor_data):
        """
        Display sensor information in a formatted way
        
        Args:
            sensor_data: List of (sensor, distance, angle_deg) tuples
        """
        print("\n" + "="*80)
        print("TOF SENSOR READINGS - CLOSEST TO WALLS")
        print("="*80)
        
        # Show closest and second closest
        if len(sensor_data) >= 1:
            closest = sensor_data[0]
            print(f"🏆 CLOSEST TO WALL:")
            print(f"   Address: 0x{closest[0].address:02x}")
            print(f"   Angle: {closest[2]:6.1f}°")
            print(f"   Distance: {closest[1]:4d} mm")
            print(f"   Position: {self._get_position_name(closest[2])}")
        
        if len(sensor_data) >= 2:
            second = sensor_data[1]
            print(f"\n🥈 SECOND CLOSEST:")
            print(f"   Address: 0x{second[0].address:02x}")
            print(f"   Angle: {second[2]:6.1f}°")
            print(f"   Distance: {second[1]:4d} mm")
            print(f"   Position: {self._get_position_name(second[2])}")
        
        # Show all sensors
        print(f"\n📊 ALL SENSORS (sorted by distance):")
        print("-" * 80)
        print(f"{'Rank':<4} {'Address':<8} {'Angle':<8} {'Distance':<10} {'Position':<12} {'Status'}")
        print("-" * 80)
        
        for i, (sensor, distance, angle_deg) in enumerate(sensor_data):
            rank = i + 1
            address = f"0x{sensor.address:02x}"
            angle_str = f"{angle_deg:6.1f}°"
            distance_str = f"{distance:4d} mm"
            position = self._get_position_name(angle_deg)
            
            # Determine status
            if distance < 100:
                status = "🔴 VERY CLOSE"
            elif distance < 200:
                status = "🟡 CLOSE"
            elif distance < 500:
                status = "🟢 MEDIUM"
            else:
                status = "🔵 FAR"
            
            print(f"{rank:<4} {address:<8} {angle_str:<8} {distance_str:<10} {position:<12} {status}")
    
    def _get_position_name(self, angle_deg):
        """
        Get a human-readable position name for an angle
        
        Args:
            angle_deg: Angle in degrees
            
        Returns:
            str: Position name
        """
        # Normalize angle to 0-360 range
        angle = angle_deg % 360
        
        if 337.5 <= angle or angle < 22.5:
            return "FRONT"
        elif 22.5 <= angle < 67.5:
            return "FRONT-RIGHT"
        elif 67.5 <= angle < 112.5:
            return "RIGHT"
        elif 112.5 <= angle < 157.5:
            return "BACK-RIGHT"
        elif 157.5 <= angle < 202.5:
            return "BACK"
        elif 202.5 <= angle < 247.5:
            return "BACK-LEFT"
        elif 247.5 <= angle < 292.5:
            return "LEFT"
        elif 292.5 <= angle < 337.5:
            return "FRONT-LEFT"
        else:
            return "UNKNOWN"
    
    def display_simple_info(self, sensor_data):
        """
        Display only the closest sensor information in a simple format
        
        Args:
            sensor_data: List of (sensor, distance, angle_deg) tuples
        """
        if len(sensor_data) >= 1:
            closest = sensor_data[0]
            print(f"Closest: 0x{closest[0].address:02x} - {closest[1]:4d}mm - {closest[2]:6.1f}°")
        else:
            print("No sensors available")
    
    def display_visual_map(self, sensor_data):
        """
        Display a simple visual map of sensor positions
        
        Args:
            sensor_data: List of (sensor, distance, angle_deg) tuples
        """
        print("\n" + "="*60)
        print("VISUAL SENSOR MAP (Top View)")
        print("="*60)
        
        # Create a simple ASCII map
        map_size = 20
        center = map_size // 2
        
        # Initialize map
        visual_map = [[' ' for _ in range(map_size)] for _ in range(map_size)]
        
        # Place robot center
        visual_map[center][center] = 'R'
        
        # Place sensors on the map
        for sensor, distance, angle_deg in sensor_data:
            # Convert angle to radians
            angle_rad = math.radians(angle_deg)
            
            # Calculate position on map (scaled)
            scale = 8  # Scale factor for visibility
            x = center + int(scale * math.cos(angle_rad))
            y = center + int(scale * math.sin(angle_rad))
            
            # Clamp to map bounds
            x = max(0, min(map_size-1, x))
            y = max(0, min(map_size-1, y))
            
            # Place sensor marker
            if distance < 200:
                visual_map[y][x] = '●'  # Close sensor
            elif distance < 500:
                visual_map[y][x] = '○'  # Medium distance
            else:
                visual_map[y][x] = '◯'  # Far sensor
        
        # Print the map
        for row in visual_map:
            print(''.join(row))
        
        print("\nLegend: R=Robot, ●=Close (<200mm), ○=Medium (200-500mm), ◯=Far (>500mm)")
    
    def run_continuous(self, update_interval=0.5):
        """
        Run continuous sensor monitoring
        
        Args:
            update_interval: Time between updates in seconds
        """
        if not self.initialize_sensors():
            return
        
        self.running = True
        print(f"\n🔄 Starting continuous TOF monitoring...")
        print(f"📊 Update interval: {update_interval}s")
        print(f"🛑 Press Ctrl+C to stop")
        
        try:
            while self.running:
                # Get sensor rankings
                sensor_data = self.get_sensor_ranking()
                
                # Display information
                self.display_sensor_info(sensor_data)
                self.display_visual_map(sensor_data)
                
                # Wait for next update
                time.sleep(update_interval)
                
        except KeyboardInterrupt:
            print(f"\n🛑 Stopping TOF monitoring...")
            self.running = False
        except Exception as e:
            print(f"❌ Error during monitoring: {e}")
            self.running = False
    
    def run_simple_continuous(self, update_interval=0.5):
        """
        Run simple continuous sensor monitoring - shows only closest sensor
        
        Args:
            update_interval: Time between updates in seconds
        """
        if not self.initialize_sensors():
            return
        
        self.running = True
        print(f"\n🔄 Starting simple TOF monitoring...")
        print(f"📊 Update interval: {update_interval}s")
        print(f"🛑 Press Ctrl+C to stop")
        
        try:
            while self.running:
                # Get sensor rankings
                sensor_data = self.get_sensor_ranking()
                
                # Display only closest sensor info
                self.display_simple_info(sensor_data)
                
                # Wait for next update
                time.sleep(update_interval)
                
        except KeyboardInterrupt:
            print(f"\n🛑 Stopping simple TOF monitoring...")
            self.running = False
        except Exception as e:
            print(f"❌ Error during monitoring: {e}")
            self.running = False
    
    def run_single_reading(self):
        """Run a single reading and display results"""
        if not self.initialize_sensors():
            return
        
        print(f"\n📊 Taking single TOF reading...")
        
        # Get sensor rankings
        sensor_data = self.get_sensor_ranking()
        
        # Display information
        self.display_sensor_info(sensor_data)
        self.display_visual_map(sensor_data)


def main():
    """Main function"""
    print("TOF Sensor Identifier")
    print("====================")
    print("This script helps identify which TOF sensor is positioned where.")
    print("It continuously reads sensors and shows which ones are closest to walls.")
    print("\nUsage:")
    print("  python tof_sensor_identifier.py           # Full continuous monitoring")
    print("  python tof_sensor_identifier.py --simple # Simple mode (closest sensor only)")
    print("  python tof_sensor_identifier.py --single # Single reading")
    
    identifier = TOFIdentifier()
    
    # Check command line arguments
    if len(sys.argv) > 1 and sys.argv[1] == "--single":
        # Single reading mode
        identifier.run_single_reading()
    elif len(sys.argv) > 1 and sys.argv[1] == "--simple":
        # Simple continuous monitoring mode
        identifier.run_simple_continuous(update_interval=0.5)  # 0.5 second updates
    else:
        # Full continuous monitoring mode
        identifier.run_continuous(update_interval=1.0)  # 1 second updates


if __name__ == "__main__":
    main()
