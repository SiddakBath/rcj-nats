"""
Localization system for the Soccer Robot
Uses TOF sensors and IMU to triangulate robot position using real measurements
"""

import math
import numpy as np
from config import LOCALIZATION_CONFIG, TOF_CONFIG
from tof_sensor import TOFManager
from imu_sensor import IMUSensor


class Localizer:
    """Localization system using TOF sensors and IMU for position triangulation"""
    
    def __init__(self, i2c_bus=None, tof_manager=None, imu_sensor=None):
        """
        Initialize localization system
        
        Args:
            i2c_bus: I2C bus instance
            tof_manager: TOFManager instance (if None, creates new one)
            imu_sensor: IMUSensor instance (if None, creates new one)
        """
        self.i2c = i2c_bus
        self.tof_manager = tof_manager if tof_manager is not None else TOFManager(i2c_bus)
        self.imu_sensor = imu_sensor if imu_sensor is not None else IMUSensor(i2c_bus)
        
        # Reset IMU initial heading when localization starts
        if self.imu_sensor.is_available():
            self.imu_sensor.reset_initial_heading()
            print("IMU initial heading reset for localization")
        
        # Localization state
        self.position = [0, 0]  # Current position estimate [x, y] in mm
        self.angle = 0.0  # Current angle estimate in radians
        self.initialized = False  # Whether we have a valid position estimate
        
        # Field configuration
        self.field_width = LOCALIZATION_CONFIG["field_width"]
        self.field_height = LOCALIZATION_CONFIG["field_height"]
        
        # Localization parameters
        self.max_distance = TOF_CONFIG["max_distance"]
        self.min_distance = TOF_CONFIG["min_distance"]
        
        # Field walls for ray casting
        self.walls = LOCALIZATION_CONFIG["walls"]
        
        # Sensor configuration
        self.sensor_angles = TOF_CONFIG["angles"]
        self.sensor_offsets = TOF_CONFIG["offsets"]
        
        print("Localization system initialized")
    
    def reset_imu_heading(self):
        """
        Reset the IMU initial heading to current compass reading
        This should be called when the robot is repositioned or angle gets corrupted
        """
        if self.imu_sensor.is_available():
            self.imu_sensor.reset_initial_heading()
            print("IMU initial heading reset to current compass reading")
            return True
        else:
            print("Cannot reset IMU heading - IMU not available")
            return False
    
    def get_imu_status(self):
        """
        Get current IMU status and heading information for debugging
        """
        if not self.imu_sensor.is_available():
            return {
                'available': False,
                'error': 'IMU not available'
            }
        
        try:
            # Get current compass heading
            compass_heading = self.imu_sensor.get_compass_heading()
            relative_heading = self.imu_sensor.get_relative_heading()
            
            return {
                'available': True,
                'compass_heading_deg': compass_heading,
                'relative_heading_deg': relative_heading,
                'relative_heading_rad': math.radians(relative_heading) if relative_heading is not None else None,
                'initial_heading_set': self.imu_sensor.initial_heading is not None
            }
        except Exception as e:
            return {
                'available': True,
                'error': f'IMU read error: {e}'
            }
    
    def _update_angle_from_imu(self):
        """
        Update robot angle from IMU using relative heading from start position
        
        Returns:
            float: Robot angle in radians
        """
        relative_heading_degrees = self.imu_sensor.get_relative_heading()
        if relative_heading_degrees is not None:
            return math.radians(relative_heading_degrees)
        return self.angle  # Keep current angle if IMU not available
    
    def localize(self):
        """
        Perform localization using TOF sensors and IMU triangulation
        
        Returns:
            list: [x, y] position in mm
        """
        # Update TOF sensor readings
        self.tof_manager.update_distances()
        
        # Update robot angle from IMU
        self.angle = self._update_angle_from_imu()
        
        # Perform triangulation to determine position
        new_position = self._triangulate_position()
        
        # Update position if triangulation was successful
        if new_position is not None:
            self.position = new_position
            self.initialized = True
        
        return self.position.copy()
    
    def get_position(self):
        """
        Get current position estimate
        
        Returns:
            list: [x, y] position in mm
        """
        return self.position.copy()
    
    def get_angle(self):
        """
        Get current angle estimate
        
        Returns:
            float: Robot angle in radians
        """
        return self.angle
    
    def reset_position(self, x=0, y=0):
        """
        Reset position estimate to given coordinates
        
        Args:
            x: X coordinate in mm
            y: Y coordinate in mm
        """
        self.position = [x, y]
        self.initialized = True
        print(f"Position reset to ({x}, {y})")
    
    def reset_localization(self):
        """
        Reset the entire localization system
        """
        self.position = [0, 0]
        self.angle = 0.0
        self.initialized = False
        print("Localization system reset")
    
    def get_sensor_data(self):
        """
        Get current sensor data for debugging
        
        Returns:
            dict: Sensor data including distances and angles
        """
        return {
            'tof_distances': self.tof_manager.get_all_distances(),
            'robot_angle': self.angle,
            'position': self.get_position(),
            'sensor_count': self.tof_manager.get_sensor_count(),
        }
    
    def get_localization_debug_info(self):
        """
        Get detailed debugging information about the localization process
        
        Returns:
            dict: Detailed localization debug information
        """
        # Get current sensor readings
        actual_readings = {}
        for sensor in self.tof_manager.sensors:
            actual_readings[sensor.angle] = self.tof_manager.sensor_distances[sensor.angle]
        
        return {
            'current_position': self.position.copy(),
            'current_angle_deg': math.degrees(self.angle),
            'actual_readings': actual_readings,
            'initialized': self.initialized
        }
    
    def get_closest_sensor(self):
        """
        Find the sensor with the shortest distance reading
        
        Returns:
            tuple: (sensor_angle, distance, direction_name) or (None, None, None) if no valid sensors
        """
        closest_distance = float('inf')
        closest_angle = None
        closest_direction = None
        
        for sensor in self.tof_manager.sensors:
            distance = self.tof_manager.sensor_distances[sensor.angle]
            if distance < closest_distance:
                closest_distance = distance
                closest_angle = sensor.angle
                # Convert angle to direction name
                angle_deg = math.degrees(sensor.angle)
                # Normalize angle to 0-360 range for consistent comparison
                angle_deg = angle_deg % 360
                if angle_deg < 0:
                    angle_deg += 360
                
                if 337.5 <= angle_deg or angle_deg < 22.5:
                    closest_direction = "Front"
                elif 22.5 <= angle_deg < 67.5:
                    closest_direction = "Front-Right"
                elif 67.5 <= angle_deg < 112.5:
                    closest_direction = "Right"
                elif 112.5 <= angle_deg < 157.5:
                    closest_direction = "Back-Right"
                elif 157.5 <= angle_deg < 202.5:
                    closest_direction = "Back"
                elif 202.5 <= angle_deg < 247.5:
                    closest_direction = "Back-Left"
                elif 247.5 <= angle_deg < 292.5:
                    closest_direction = "Left"
                elif 292.5 <= angle_deg < 337.5:
                    closest_direction = "Front-Left"
        
        return closest_angle, closest_distance, closest_direction
    
    def get_current_robot_state(self):
        """
        Get the current robot state
        
        Returns:
            dict: Current robot state including position, angle, and sensor data
        """
        return {
            'position': self.position.copy(),
            'angle_rad': self.angle,
            'angle_deg': math.degrees(self.angle),
            'sensor_data': self.get_sensor_data(),
            'initialized': self.initialized
        }
    
    def _ray_cast_to_walls(self, robot_x, robot_y, sensor_angle):
        """
        Cast a ray from robot position at given angle to find intersection with field walls
        
        Args:
            robot_x: Robot X position in mm
            robot_y: Robot Y position in mm  
            sensor_angle: Sensor angle in radians (relative to robot front)
            
        Returns:
            float: Distance to wall intersection in mm, or None if no intersection
        """
        # Calculate absolute sensor direction (robot angle + sensor angle)
        absolute_angle = self.angle + sensor_angle
        
        # Ray direction vector
        dx = math.cos(absolute_angle)
        dy = math.sin(absolute_angle)
        
        min_distance = float('inf')
        
        # Check intersection with each wall
        for wall in self.walls:
            if wall['type'] == 'vertical':
                # Vertical wall: x = wall_x
                wall_x = wall['x']
                if abs(dx) < 1e-6:  # Ray is parallel to wall
                    continue
                    
                # Calculate intersection point
                t = (wall_x - robot_x) / dx
                if t > 0:  # Ray goes forward
                    intersection_y = robot_y + t * dy
                    
                    # Check if intersection is within wall bounds
                    if wall['y_min'] <= intersection_y <= wall['y_max']:
                        distance = t
                        min_distance = min(min_distance, distance)
                        
            elif wall['type'] == 'horizontal':
                # Horizontal wall: y = wall_y
                wall_y = wall['y']
                if abs(dy) < 1e-6:  # Ray is parallel to wall
                    continue
                    
                # Calculate intersection point
                t = (wall_y - robot_y) / dy
                if t > 0:  # Ray goes forward
                    intersection_x = robot_x + t * dx
                    
                    # Check if intersection is within wall bounds
                    if wall['x_min'] <= intersection_x <= wall['x_max']:
                        distance = t
                        min_distance = min(min_distance, distance)
        
        return min_distance if min_distance != float('inf') else None
    
    def _triangulate_position(self):
        """
        Triangulate robot position using TOF sensor readings
        This is the core method that solves for position without circular dependencies
        
        Returns:
            list: [x, y] position in mm, or None if triangulation fails
        """
        # Get valid sensor readings
        valid_readings = []
        
        for sensor in self.tof_manager.sensors:
            distance = self.tof_manager.sensor_distances[sensor.angle]
            
            # Check if reading is within valid range
            if (self.min_distance <= distance <= self.max_distance):
                valid_readings.append({
                    'angle': sensor.angle,
                    'distance': distance,
                    'offset': sensor.offset
                })
        
        if len(valid_readings) < 3:
            print(f"Not enough valid sensor readings for triangulation: {len(valid_readings)}")
            return None
        
        # Use geometric triangulation approach
        # We'll use the fact that each sensor reading gives us a circle of possible positions
        # The intersection of these circles gives us the robot position
        
        best_position = None
        best_error = float('inf')
        
        # Try different combinations of sensors for triangulation
        from itertools import combinations
        
        for sensor_combo in combinations(valid_readings, 3):
            position = self._solve_position_from_sensors_geometric(sensor_combo)
            if position is not None:
                # Calculate error for this position
                error = self._calculate_position_error(position, valid_readings)
                if error < best_error:
                    best_error = error
                    best_position = position
        
        if best_position is not None:
            print(f"Triangulation successful: position ({best_position[0]:.1f}, {best_position[1]:.1f}), error: {best_error:.1f}")
        
        return best_position
    
    def _solve_position_from_sensors_geometric(self, sensors):
        """
        Solve robot position using 3 sensor readings with geometric approach
        This method avoids circular dependencies by using a different approach
        
        Args:
            sensors: List of 3 sensor readings with angle, distance, offset
            
        Returns:
            list: [x, y] position in mm, or None if solution fails
        """
        try:
            # For each sensor, we know:
            # 1. The sensor's relative angle from robot front
            # 2. The distance reading to a wall
            # 3. The sensor's offset from robot center
            
            # We need to find the robot position such that:
            # - Each sensor's distance reading matches the expected distance to walls
            
            # Use iterative approach to solve for robot position
            # Start with a reasonable initial guess (field center)
            initial_guess = [self.field_width / 2, self.field_height / 2]
            
            # Use optimization to find position that minimizes error
            position = self._optimize_position(initial_guess, sensors)
            
            if position is not None:
                # Validate the solution
                if self._validate_position(position, sensors):
                    return position
            
        except Exception as e:
            print(f"Error in geometric position solving: {e}")
        
        return None
    
    def _optimize_position(self, initial_guess, sensors):
        """
        Optimize robot position using sensor readings
        Uses a grid search approach to find the best position
        
        Args:
            initial_guess: Initial position guess [x, y]
            sensors: List of sensor readings
            
        Returns:
            list: [x, y] position or None if optimization fails
        """
        # Grid search parameters
        search_range = 500  # Search within 500mm of initial guess
        grid_size = 50  # 50mm grid resolution
        
        best_position = None
        best_error = float('inf')
        
        # Search in a grid around the initial guess
        for dx in range(-search_range, search_range + 1, grid_size):
            for dy in range(-search_range, search_range + 1, grid_size):
                test_x = initial_guess[0] + dx
                test_y = initial_guess[1] + dy
                
                # Check if position is within field bounds
                if (0 <= test_x <= self.field_width and 
                    0 <= test_y <= self.field_height):
                    
                    # Calculate error for this position
                    error = self._calculate_position_error([test_x, test_y], sensors)
                    
                    if error < best_error:
                        best_error = error
                        best_position = [test_x, test_y]
        
        # If we found a reasonable solution, do fine-tuning
        if best_position is not None and best_error < 1000:  # 1 meter error threshold
            # Fine-tune the position with smaller grid
            fine_position = self._fine_tune_position(best_position, sensors)
            if fine_position is not None:
                return fine_position
            return best_position
        
        return None
    
    def _fine_tune_position(self, coarse_position, sensors):
        """
        Fine-tune position using smaller grid search
        
        Args:
            coarse_position: Coarse position estimate
            sensors: List of sensor readings
            
        Returns:
            list: Fine-tuned position or None
        """
        fine_range = 100  # 100mm fine search range
        fine_grid = 10  # 10mm fine grid resolution
        
        best_position = None
        best_error = float('inf')
        
        for dx in range(-fine_range, fine_range + 1, fine_grid):
            for dy in range(-fine_range, fine_range + 1, fine_grid):
                test_x = coarse_position[0] + dx
                test_y = coarse_position[1] + dy
                
                if (0 <= test_x <= self.field_width and 
                    0 <= test_y <= self.field_height):
                    
                    error = self._calculate_position_error([test_x, test_y], sensors)
                    
                    if error < best_error:
                        best_error = error
                        best_position = [test_x, test_y]
        
        return best_position if best_error < 500 else None  # 500mm error threshold
    
    def _calculate_position_error(self, position, all_readings):
        """
        Calculate error between expected and measured distances for a given position
        
        Args:
            position: [x, y] position to test
            all_readings: All sensor readings
            
        Returns:
            float: Total error in mm
        """
        total_error = 0
        valid_count = 0
        
        for reading in all_readings:
            # Calculate expected distance from position to walls
            expected_distance = self._ray_cast_to_walls(position[0], position[1], reading['angle'])
            
            if expected_distance is not None:
                error = abs(reading['distance'] - expected_distance)
                total_error += error
                valid_count += 1
        
        # Return average error per valid sensor
        return total_error / valid_count if valid_count > 0 else float('inf')
    
    def _validate_position(self, position, sensors):
        """
        Validate that a position is consistent with sensor readings
        
        Args:
            position: [x, y] position to validate
            sensors: List of sensor readings
            
        Returns:
            bool: True if position is valid
        """
        # Check if position is within field bounds
        if not (0 <= position[0] <= self.field_width and 
                0 <= position[1] <= self.field_height):
            return False
        
        # Check if error is reasonable
        error = self._calculate_position_error(position, sensors)
        return error < 200  # 200mm error threshold
    
    def get_localization_accuracy(self):
        """
        Get current localization accuracy based on sensor readings
        
        Returns:
            dict: Accuracy information including error estimates
        """
        if len(self.tof_manager.sensors) < 3:
            return {
                'accuracy': 'poor',
                'error_estimate': 'insufficient_sensors',
                'sensor_count': len(self.tof_manager.sensors)
            }
        
        # Calculate position error for current position
        valid_readings = []
        for sensor in self.tof_manager.sensors:
            distance = self.tof_manager.sensor_distances[sensor.angle]
            if self.min_distance <= distance <= self.max_distance:
                valid_readings.append({
                    'angle': sensor.angle,
                    'distance': distance
                })
        
        if len(valid_readings) < 3:
            return {
                'accuracy': 'poor',
                'error_estimate': 'insufficient_valid_readings',
                'valid_sensor_count': len(valid_readings)
            }
        
        # Calculate error for current position
        current_error = self._calculate_position_error(self.position, valid_readings)
        avg_error = current_error
        
        # Determine accuracy level
        if avg_error < 50:
            accuracy = 'excellent'
        elif avg_error < 100:
            accuracy = 'good'
        elif avg_error < 200:
            accuracy = 'fair'
        else:
            accuracy = 'poor'
        
        return {
            'accuracy': accuracy,
            'error_estimate': avg_error,
            'total_error': current_error,
            'valid_sensor_count': len(valid_readings),
            'sensor_count': len(self.tof_manager.sensors)
        }
    
    def get_field_bounds_check(self):
        """
        Check if current position is within field bounds
        
        Returns:
            dict: Bounds check information
        """
        x, y = self.position
        
        within_bounds = (0 <= x <= self.field_width and 
                        0 <= y <= self.field_height)
        
        return {
            'within_bounds': within_bounds,
            'position': self.position.copy(),
            'field_bounds': {
                'width': self.field_width,
                'height': self.field_height
            },
            'distance_to_edges': {
                'left': x,
                'right': self.field_width - x,
                'bottom': y,
                'top': self.field_height - y
            }
        }
    
    def get_sensor_health_status(self):
        """
        Get health status of all TOF sensors
        
        Returns:
            dict: Sensor health information
        """
        sensor_status = []
        healthy_count = 0
        
        for sensor in self.tof_manager.sensors:
            distance = self.tof_manager.sensor_distances[sensor.angle]
            is_healthy = self.min_distance <= distance <= self.max_distance
            
            if is_healthy:
                healthy_count += 1
            
            sensor_status.append({
                'angle_degrees': math.degrees(sensor.angle),
                'distance': distance,
                'healthy': is_healthy,
                'address': sensor.address
            })
        
        return {
            'total_sensors': len(self.tof_manager.sensors),
            'healthy_sensors': healthy_count,
            'sensor_status': sensor_status,
            'health_percentage': (healthy_count / len(self.tof_manager.sensors)) * 100 if self.tof_manager.sensors else 0
        }
    
    def test_localization_system(self):
        """
        Test the localization system with known positions
        
        Returns:
            dict: Test results and diagnostics
        """
        print("Testing localization system...")
        
        # Test known positions
        test_positions = [
            [1215, 910],  # Center of field
            [500, 500],   # Near corner
            [2000, 1500], # Near opposite corner
            [1215, 100],  # Near bottom wall
            [1215, 1720], # Near top wall
        ]
        
        results = []
        
        for test_pos in test_positions:
            print(f"Testing position: ({test_pos[0]}, {test_pos[1]})")
            
            # Set position and test ray casting
            original_pos = self.position.copy()
            self.position = test_pos.copy()
            
            # Test ray casting for each sensor
            ray_cast_results = {}
            for sensor in self.tof_manager.sensors:
                distance = self._ray_cast_to_walls(test_pos[0], test_pos[1], sensor.angle)
                ray_cast_results[math.degrees(sensor.angle)] = distance
            
            # Restore original position
            self.position = original_pos
            
            results.append({
                'test_position': test_pos,
                'ray_cast_distances': ray_cast_results,
                'sensor_count': len(ray_cast_results)
            })
        
        # Test triangulation with current readings
        current_accuracy = self.get_localization_accuracy()
        sensor_health = self.get_sensor_health_status()
        bounds_check = self.get_field_bounds_check()
        
        return {
            'test_results': results,
            'current_accuracy': current_accuracy,
            'sensor_health': sensor_health,
            'bounds_check': bounds_check,
            'system_status': 'operational' if sensor_health['health_percentage'] > 50 else 'degraded'
        }