"""
Time-of-Flight (TOF) sensor module for the Soccer Robot
Handles VL53L0X TOF sensors for distance measurement
"""

import time
import math
import board
import busio
from config import TOF_CONFIG


class TOFSensor:
    """Handles VL53L0X TOF sensor for distance measurement"""
    
    def __init__(self, address, offset=(0, 0), angle=0, i2c_bus=None):
        """
        Initialize TOF sensor
        
        Args:
            address: I2C address of the TOF sensor
            offset: (x, y) offset from robot center in mm
            angle: Angle of sensor relative to robot front in radians
            i2c_bus: I2C bus instance (if None, creates new one)
        """
        self.address = address
        self.offset = offset
        self.angle = angle
        self.i2c = i2c_bus if i2c_bus is not None else busio.I2C(board.SCL, board.SDA)
        
        # Distance limits from config
        self.max_distance = TOF_CONFIG["max_distance"]
        self.min_distance = TOF_CONFIG["min_distance"]
        self.read_register = TOF_CONFIG["read_register"]
        
        # State tracking
        self.last_seq = None
        self.last_distance = 0
        
        # Initialize sensor
        self.setup_sensor()
    
    def setup_sensor(self):
        """Initialize the TOF sensor"""
        try:
            # Test I2C communication
            self._test_communication()
            print(f"TOF sensor at 0x{self.address:02x} initialized successfully")
        except Exception as e:
            print(f"Failed to initialize TOF sensor at 0x{self.address:02x}: {e}")
            raise
    
    def _test_communication(self):
        """Test I2C communication with the sensor"""
        try:
            # Try to read from the sensor
            write_buf = bytes([self.read_register])
            read_buf = bytearray(5)
            self.i2c.writeto(self.address, write_buf)
            self.i2c.readfrom_into(self.address, read_buf)
        except Exception as e:
            raise Exception(f"I2C communication failed: {e}")
    
    def _read_distance(self, wait_for_new=True):
        """
        Read distance from TOF sensor
        
        Args:
            wait_for_new: If True, wait for new data point
            
        Returns:
            int: Distance in mm
        """
        write_buf = bytes([self.read_register])
        read_buf = bytearray(5)
        
        while True:
            try:
                self.i2c.writeto(self.address, write_buf)
                self.i2c.readfrom_into(self.address, read_buf)
                
                # Extract sequence number and distance
                seq = read_buf[0]
                distance = (read_buf[1] | read_buf[2] << 8 | 
                           read_buf[3] << 16 | read_buf[4] << 24)
                
                # Check if this is new data
                changed = seq != self.last_seq
                
                # If we want fresh data and this isn't new, continue
                if changed or not wait_for_new:
                    self.last_seq = seq
                    
                    # Validate distance
                    if (self.min_distance <= distance <= self.max_distance):
                        self.last_distance = distance
                    # If distance is invalid, return last valid distance
                    
                    return self.last_distance
                
                # Small delay to avoid busy waiting
                time.sleep(0.00001)
                
            except Exception as e:
                print(f"Error reading TOF sensor at 0x{self.address:02x}: {e}")
                return self.last_distance
    
    def get_distance(self, wait_for_new=True):
        """
        Get current distance measurement
        
        Args:
            wait_for_new: If True, wait for new data point
            
        Returns:
            int: Distance in mm
        """
        return self._read_distance(wait_for_new)
    
    def get_angle_degrees(self):
        """
        Get sensor angle in degrees
        
        Returns:
            float: Angle in degrees
        """
        return math.degrees(self.angle)
    
    def get_offset(self):
        """
        Get sensor offset from robot center
        
        Returns:
            tuple: (x, y) offset in mm
        """
        return self.offset
    
    def is_valid_distance(self, distance):
        """
        Check if distance measurement is valid
        
        Args:
            distance: Distance to validate
            
        Returns:
            bool: True if distance is valid
        """
        return self.min_distance <= distance <= self.max_distance


class TOFManager:
    """Manages multiple TOF sensors for localization"""
    
    def __init__(self, i2c_bus=None):
        """
        Initialize TOF manager
        
        Args:
            i2c_bus: I2C bus instance (if None, creates new one)
        """
        self.i2c = i2c_bus if i2c_bus is not None else busio.I2C(board.SCL, board.SDA)
        self.sensors = []
        self.sensor_angles = []
        self.sensor_distances = {}
        
        # Initialize sensors from config
        self.setup_sensors()
    
    def setup_sensors(self):
        """Initialize all TOF sensors from configuration"""
        print("Initializing TOF sensors...")
        
        for i, address in enumerate(TOF_CONFIG["addresses"]):
            try:
                # Get angle in radians
                angle_degrees = TOF_CONFIG["angles"][i]
                angle_radians = math.radians(angle_degrees)
                
                # Get offset
                offset = TOF_CONFIG["offsets"][address]
                
                # Create sensor
                sensor = TOFSensor(
                    address=address,
                    offset=offset,
                    angle=angle_radians,
                    i2c_bus=self.i2c
                )
                
                self.sensors.append(sensor)
                self.sensor_angles.append(angle_radians)
                self.sensor_distances[angle_radians] = 0  # Initialize with 0
                
                print(f"  ✅ TOF sensor at 0x{address:02x}, angle {angle_degrees}°")
                
            except Exception as e:
                print(f"  ❌ Failed to initialize TOF at 0x{address:02x}: {e}")
        
        print(f"Initialized {len(self.sensors)} TOF sensors")
    
    def update_distances(self):
        """Update all sensor distance readings"""
        for sensor in self.sensors:
            distance = sensor.get_distance(wait_for_new=True)
            self.sensor_distances[sensor.angle] = distance
    
    def get_distance_at_angle(self, angle):
        """
        Get distance measurement at specific angle
        
        Args:
            angle: Angle in radians
            
        Returns:
            int: Distance in mm, or 0 if no sensor at that angle
        """
        return self.sensor_distances.get(angle, 0)
    
    def get_all_distances(self):
        """
        Get all distance measurements
        
        Returns:
            dict: Angle -> distance mapping
        """
        return self.sensor_distances.copy()
    
    def get_sensor_count(self):
        """
        Get number of active sensors
        
        Returns:
            int: Number of sensors
        """
        return len(self.sensors)
    
    def get_sensor_info(self):
        """
        Get information about all sensors
        
        Returns:
            list: List of sensor information dictionaries
        """
        info = []
        for sensor in self.sensors:
            info.append({
                'address': sensor.address,
                'angle_degrees': sensor.get_angle_degrees(),
                'angle_radians': sensor.angle,
                'offset': sensor.get_offset(),
                'distance': self.sensor_distances[sensor.angle]
            })
        return info


# Test code
if __name__ == "__main__":
    print("Testing TOF sensors...")
    
    try:
        # Create TOF manager
        tof_manager = TOFManager()
        
        if tof_manager.get_sensor_count() == 0:
            print("No TOF sensors found!")
            exit(1)
        
        print(f"Found {tof_manager.get_sensor_count()} TOF sensors")
        print("Reading distances (press Ctrl+C to stop)...")
        
        while True:
            # Update all distances
            tof_manager.update_distances()
            
            # Print current readings
            print("\nTOF Readings:")
            for sensor in tof_manager.sensors:
                distance = tof_manager.sensor_distances[sensor.angle]
                angle_deg = sensor.get_angle_degrees()
                print(f"  Angle {angle_deg:6.1f}°: {distance:4d} mm")
            
            time.sleep(0.1)  # 10 Hz update rate
            
    except KeyboardInterrupt:
        print("\nStopping TOF sensor test...")
    except Exception as e:
        print(f"Error in TOF sensor test: {e}")
