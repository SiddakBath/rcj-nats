"""
Localization system for the Soccer Robot
Uses TOF sensors and IMU to triangulate robot position using real measurements
"""

import math
import time
from config import LOCALIZATION_CONFIG, TOF_CONFIG
from tof_sensor import TOFManager
from imu_sensor import IMUSensor

# Try to import numpy, fall back to basic math if not available
try:
    import numpy as np
    HAS_NUMPY = True
except ImportError:
    HAS_NUMPY = False
    print("Warning: numpy not available, using basic math operations")


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
        
        # Localization state
        self.position = [0, 0]  # Current position estimate [x, y] in mm
        self.angle = 0.0  # Current angle estimate in radians
        self.confidence = 0.0  # Confidence in current estimate (0-1)
        
        # Smoothing/filtering for stable estimates
        self.position_history = []  # History of position estimates for smoothing
        self.angle_history = []    # History of angle estimates for smoothing
        self.confidence_history = []  # History of confidence estimates for smoothing
        self.max_history_length = 5  # Number of recent estimates to keep for smoothing
        
        # Field configuration
        self.field_width = LOCALIZATION_CONFIG["field_width"]
        self.field_height = LOCALIZATION_CONFIG["field_height"]
        
        # Ray casting parameters
        self.min_sensors = 3  # Minimum sensors needed for localization
        self.max_distance = TOF_CONFIG["max_distance"]
        self.min_distance = TOF_CONFIG["min_distance"]
        
        # Grid search parameters
        self.grid_resolution = LOCALIZATION_CONFIG.get("grid_resolution", 50)  # mm per grid cell
        self.max_search_radius = LOCALIZATION_CONFIG.get("max_search_radius", 500)  # mm
        
        # Optimization parameters
        self.best_guess = [0, 0]  # Best position estimate
        self.best_error = float('inf')  # Best error score
        
        # Field walls for ray casting
        self.walls = LOCALIZATION_CONFIG["walls"]
        
        print("Grid-based localization system initialized")
    
    def _cast_ray(self, position, angle):
        """
        Cast a ray from position at angle and find distance to closest wall
        
        Args:
            position: [x, y] position in mm
            angle: Ray angle in radians
            
        Returns:
            float: Distance to closest wall intersection
        """
        # Check if walls list is empty
        if not self.walls:
            print("Warning: No walls defined for ray casting")
            return self.max_distance
        
        minimum_distance = float('inf')
        dx = math.cos(angle)
        dy = math.sin(angle)
        
        for wall in self.walls:
            if wall['type'] == 'horizontal':
                # Horizontal wall: y = constant
                if abs(dy) < 1e-6:  # Ray is parallel to wall
                    continue
                    
                t = (wall['y'] - position[1]) / dy
                if t <= 0:  # Intersection is behind the ray origin
                    continue
                    
                x = position[0] + t * dx
                if x < wall['x_min'] or x > wall['x_max']:
                    continue
                    
                y = position[1] + t * dy
                dist_squared = (x - position[0])**2 + (y - position[1])**2
                if dist_squared < minimum_distance:
                    minimum_distance = dist_squared
                    
            elif wall['type'] == 'vertical':
                # Vertical wall: x = constant
                if abs(dx) < 1e-6:  # Ray is parallel to wall
                    continue
                    
                t = (wall['x'] - position[0]) / dx
                if t <= 0:  # Intersection is behind the ray origin
                    continue
                    
                y = position[1] + t * dy
                if y < wall['y_min'] or y > wall['y_max']:
                    continue
                    
                x = position[0] + t * dx
                dist_squared = (x - position[0])**2 + (y - position[1])**2
                if dist_squared < minimum_distance:
                    minimum_distance = dist_squared
        
        # Handle case where no walls are hit
        if minimum_distance == float('inf'):
            return self.max_distance
        
        return math.sqrt(minimum_distance)
    
    def _compute_expected_readings_for_position(self, position, robot_angle):
        """
        Compute expected sensor readings for a specific position and robot angle
        This is the correct approach - compute on-the-fly for the actual robot angle
        
        Args:
            position: [x, y] position in mm
            robot_angle: Robot heading angle in radians (continuous, not discretized)
            
        Returns:
            dict: Angle -> expected distance mapping
        """
        expected_readings = {}
        for sensor in self.tof_manager.sensors:
            # Calculate world angle for this sensor
            world_angle = robot_angle + sensor.angle
            # Cast ray to get expected distance
            expected_distance = self._cast_ray(position, world_angle)
            expected_readings[sensor.angle] = expected_distance
        
        return expected_readings
    
    def _get_expected_readings(self, position, angle):
        """
        Get expected sensor readings by computing them on-the-fly for the actual robot angle
        This is the correct approach - no lookup table needed for continuous angles
        
        Args:
            position: [x, y] position in mm
            angle: Robot heading angle in radians (continuous)
            
        Returns:
            dict: Angle -> expected distance mapping
        """
        return self._compute_expected_readings_for_position(position, angle)
    
    def _log_sensor_table(self):
        """
        Log expected vs actual sensor readings table for debugging
        Shows what the algorithm is comparing to verify values look correct
        """
        print("\n" + "="*80)
        print("SENSOR READINGS TABLE (Expected vs Actual)")
        print("="*80)
        
        # Get current position and angle
        current_pos = self.position
        current_angle = self.angle
        
        # Get expected readings for current position
        expected_readings = self._compute_expected_readings_for_position(current_pos, current_angle)
        
        # Get actual sensor readings
        actual_readings = {}
        for sensor in self.tof_manager.sensors:
            actual_readings[sensor.angle] = self.tof_manager.sensor_distances[sensor.angle]
        
        print(f"Robot Position: ({current_pos[0]:.1f}, {current_pos[1]:.1f}) mm")
        print(f"Robot Angle: {math.degrees(current_angle):.1f}°")
        print(f"Current Error: {self.best_error:.1f}")
        print()
        
        # Print table header
        print(f"{'Sensor':<12} {'Angle':<8} {'Expected':<10} {'Actual':<10} {'Diff':<8} {'Status':<6}")
        print("-" * 70)
        
        # Print each sensor
        for sensor in self.tof_manager.sensors:
            angle_deg = math.degrees(sensor.angle)
            expected = expected_readings.get(sensor.angle, 0)
            actual = actual_readings.get(sensor.angle, 0)
            diff = abs(expected - actual)
            
            # Determine sensor status
            if self.min_distance <= actual <= self.max_distance:
                status = "✓"
            else:
                status = "✗"
            
            # Get sensor direction name
            if -112.5 <= angle_deg < -67.5:
                direction = "Left"
            elif -67.5 <= angle_deg < -22.5:
                direction = "Front-Left"
            elif -22.5 <= angle_deg < 22.5:
                direction = "Front"
            elif 22.5 <= angle_deg < 67.5:
                direction = "Front-Right"
            elif 67.5 <= angle_deg < 112.5:
                direction = "Right"
            elif 112.5 <= angle_deg < 157.5:
                direction = "Back-Right"
            elif 157.5 <= angle_deg < 202.5:
                direction = "Back"
            elif 202.5 <= angle_deg < 247.5:
                direction = "Back-Left"
            else:
                direction = "Front-Left"
            
            print(f"{direction:<12} {angle_deg:>6.1f}° {expected:>8.1f}mm {actual:>8.1f}mm {diff:>6.1f}mm {status:>4}")
        
        print("-" * 70)
        
        # Summary statistics
        valid_sensors = sum(1 for d in actual_readings.values() 
                           if self.min_distance <= d <= self.max_distance)
        total_sensors = len(actual_readings)
        avg_error = sum(abs(expected_readings.get(angle, 0) - actual_readings.get(angle, 0)) 
                       for angle in expected_readings.keys()) / len(expected_readings)
        
        print(f"Valid Sensors: {valid_sensors}/{total_sensors}")
        print(f"Average Error: {avg_error:.1f}mm")
        print(f"Confidence: {self.confidence:.3f}")
        print("="*80 + "\n")
    
    def log_sensor_table_now(self):
        """
        Manually trigger sensor table logging for immediate debugging
        """
        self._log_sensor_table()
    
    def _compute_error(self, position, bot_angle):
        """
        Compute error between expected and actual TOF measurements
        This implements step 4 of the localization framework using the exact approach described
        
        Args:
            position: [x, y] position in mm
            bot_angle: Robot heading angle in radians
            
        Returns:
            float: Total error (sum of absolute differences)
        """
        error = 0.0
        valid_sensors = 0
        expected_readings = self._get_expected_readings(position, bot_angle)
        
        # Get actual sensor readings
        actual_readings = {}
        for sensor in self.tof_manager.sensors:
            actual_readings[sensor.angle] = self.tof_manager.sensor_distances[sensor.angle]
        
        # Compare actual vs expected readings for each sensor
        for sensor_angle in expected_readings:
            actual_distance = actual_readings.get(sensor_angle, self.max_distance)
            expected_distance = expected_readings[sensor_angle]
            
            # Only include valid measurements in error calculation
            if self.min_distance <= actual_distance <= self.max_distance:
                # Use absolute difference as described in the methodology
                diff = abs(actual_distance - expected_distance)
                
                # Apply improved distance-based weighting to prevent overfitting
                # Cap weights to prevent overemphasis of very close sensors
                base_weight = 0.5 + 0.5 / (1.0 + actual_distance / 1000.0)
                weight = min(base_weight, 1.0)  # Cap at 1.0 to prevent overfitting
                error += weight * diff
                valid_sensors += 1
        
        # Return total error (not normalized) to match the described approach
        # The algorithm should find the position with minimum total error
        return error
    
    def _apply_smoothing(self, new_position, new_angle, new_confidence):
        """
        Apply smoothing/filtering to position and angle estimates for stability
        This implements step 6 of the localization framework with smoothing
        
        Args:
            new_position: [x, y] new position estimate in mm
            new_angle: New angle estimate in radians
            new_confidence: New confidence value (0-1)
            
        Returns:
            tuple: (smoothed_position, smoothed_angle, smoothed_confidence)
        """
        # Add new estimate to history
        self.position_history.append(new_position.copy())
        self.angle_history.append(new_angle)
        self.confidence_history.append(new_confidence)
        
        # Keep only recent history
        if len(self.position_history) > self.max_history_length:
            self.position_history.pop(0)
            self.angle_history.pop(0)
            self.confidence_history.pop(0)
        
        # Apply smoothing only if we have enough history and good confidence
        if len(self.position_history) >= 3 and new_confidence > 0.3:
            # Use weighted average with more weight on recent estimates
            weights = [0.1, 0.2, 0.3, 0.4, 0.5][-len(self.position_history):]
            
            # Smooth position
            smoothed_x = sum(pos[0] * w for pos, w in zip(self.position_history, weights)) / sum(weights)
            smoothed_y = sum(pos[1] * w for pos, w in zip(self.position_history, weights)) / sum(weights)
            smoothed_position = [smoothed_x, smoothed_y]
            
            # Smooth angle using proper circular statistics to avoid wrap-around issues
            sin_sum = sum(math.sin(angle) * weight for angle, weight in zip(self.angle_history, weights))
            cos_sum = sum(math.cos(angle) * weight for angle, weight in zip(self.angle_history, weights))
            smoothed_angle = math.atan2(sin_sum, cos_sum)
            
            # Smooth confidence using actual confidence history
            smoothed_confidence = sum(c * w for c, w in zip(self.confidence_history, weights)) / sum(weights)
            
            return smoothed_position, smoothed_angle, smoothed_confidence
        else:
            # Not enough history or low confidence, return current values
            return new_position.copy(), new_angle, new_confidence
    
    def _optimize_position(self, bot_angle):
        """
        Optimize position using coarse-to-fine iterative grid search
        This implements step 5 of the localization framework with proper coarse-to-fine approach
        
        Args:
            bot_angle: Robot heading angle in radians
            
        Returns:
            tuple: (position, error) where position is [x, y] in mm
        """
        # No lookup table needed - compute expected readings on-the-fly for actual robot angle
        
        # Start with current best guess or field center
        if self.best_error == float('inf'):
            self.best_guess = [self.field_width / 2, self.field_height / 2]
            self.best_error = self._compute_error(self.best_guess, bot_angle)
        
        # Coarse-to-fine grid search with multiple resolution levels
        resolution_levels = [
            self.grid_resolution * 8,   # Very coarse (400mm)
            self.grid_resolution * 4,   # Coarse (200mm) 
            self.grid_resolution * 2,   # Medium (100mm)
            self.grid_resolution       # Fine (50mm)
        ]
        
        for current_resolution in resolution_levels:
            converged = False
            iterations = 0
            max_iterations = 5  # Fewer iterations per level for efficiency
            
            while not converged and iterations < max_iterations:
                converged = True
                iterations += 1
                
                # Search in a grid around current best guess
                search_range = max(1, int(current_resolution / self.grid_resolution))
                
                for dx in range(-search_range, search_range + 1):
                    for dy in range(-search_range, search_range + 1):
                        guess_pos = [
                            self.best_guess[0] + dx * current_resolution,
                            self.best_guess[1] + dy * current_resolution
                        ]
                        
                        # Keep position within field bounds
                        guess_pos[0] = max(0, min(self.field_width - 1, guess_pos[0]))
                        guess_pos[1] = max(0, min(self.field_height - 1, guess_pos[1]))
                        
                        error = self._compute_error(guess_pos, bot_angle)
                        
                        if error < self.best_error:
                            converged = False
                            self.best_error = error
                            self.best_guess = guess_pos.copy()
            
            # Early termination if error is very low
            if self.best_error < 50:  # Very good match found
                break
        
        return self.best_guess.copy(), self.best_error
    
    def _global_grid_search(self, bot_angle):
        """
        Perform global grid search to find the best initial position
        This helps avoid local minima in the optimization
        
        Args:
            bot_angle: Robot heading angle in radians
            
        Returns:
            tuple: (position, error) where position is [x, y] in mm
        """
        print("Performing global grid search...")
        
        # Use coarser grid for global search
        coarse_resolution = self.grid_resolution * 4
        grid_width = int(self.field_width / coarse_resolution)
        grid_height = int(self.field_height / coarse_resolution)
        
        best_position = [self.field_width / 2, self.field_height / 2]
        best_error = float('inf')
        
        # Search entire field with coarse grid
        for x_idx in range(grid_width):
            for y_idx in range(grid_height):
                position = [
                    x_idx * coarse_resolution + coarse_resolution / 2,
                    y_idx * coarse_resolution + coarse_resolution / 2
                ]
                
                error = self._compute_error(position, bot_angle)
                
                if error < best_error:
                    best_error = error
                    best_position = position.copy()
        
        print(f"Global search found best position: {best_position}, error: {best_error}")
        return best_position, best_error
    
    
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
        Perform localization using grid-based algorithm with real TOF measurements
        This implements the complete localization framework with continuous updates
        
        Returns:
            tuple: (position, confidence) where position is [x, y] in mm
        """
        # Continuously update TOF sensor readings with real sensor data
        self.tof_manager.update_distances()
        
        # Continuously update robot angle from IMU
        self.angle = self._update_angle_from_imu()
        
        # Log continuous updates for debugging
        if hasattr(self, '_last_update_time'):
            time_since_update = time.time() - self._last_update_time
            if time_since_update > 1.0:  # Log every second
                sensor_data = self.get_continuous_sensor_data()
                print(f"[CONTINUOUS UPDATE] Angle: {sensor_data['robot_angle_deg']:.1f}°, "
                      f"Valid sensors: {sensor_data['valid_sensors']}/{sensor_data['total_sensors']}")
                self._last_update_time = time.time()
        else:
            self._last_update_time = time.time()
        
        # Log expected vs actual sensor readings every 5 seconds
        if hasattr(self, '_last_sensor_table_log_time'):
            time_since_table_log = time.time() - self._last_sensor_table_log_time
            if time_since_table_log > 5.0:  # Log every 5 seconds
                self._log_sensor_table()
                self._last_sensor_table_log_time = time.time()
        else:
            self._last_sensor_table_log_time = time.time()
        
        # Get current sensor readings for debugging
        current_sensor_readings = {}
        for sensor in self.tof_manager.sensors:
            current_sensor_readings[sensor.angle] = self.tof_manager.sensor_distances[sensor.angle]
        
        # Count valid measurements
        valid_measurements = 0
        for sensor in self.tof_manager.sensors:
            distance = self.tof_manager.sensor_distances[sensor.angle]
            if self.min_distance <= distance <= self.max_distance:
                valid_measurements += 1
        
        if valid_measurements < self.min_sensors:
            # Not enough valid measurements - return current position with low confidence
            print(f"Warning: Only {valid_measurements} valid sensors (need {self.min_sensors})")
            return self.position.copy(), 0.0
        
        # Check if we need to perform global search (first time or high error)
        if self.best_error == float('inf') or self.best_error > 3000:
            print("Performing global search for initial position...")
            position, error = self._global_grid_search(self.angle)
            self.best_guess = position.copy()
            self.best_error = error
        
        # Optimize position using iterative grid search
        position, error = self._optimize_position(self.angle)
        
        # Validate the localization result
        is_valid = self.validate_localization(position, self.angle)
        
        if not is_valid:
            print(f"Warning: Localization result failed validation (error: {error:.1f})")
            # Try global search if validation fails (but limit attempts)
            if not hasattr(self, '_validation_failures'):
                self._validation_failures = 0
            
            if self._validation_failures < 3:  # Limit to 3 attempts
                position, error = self._global_grid_search(self.angle)
                is_valid = self.validate_localization(position, self.angle)
                if not is_valid:
                    print(f"Warning: Global search also failed validation (error: {error:.1f})")
                    self._validation_failures += 1
                else:
                    self._validation_failures = 0  # Reset on success
            else:
                print("Too many validation failures, accepting current result")
                is_valid = True  # Accept the result to break the loop
                self._validation_failures = 0
        
        # Calculate confidence based on error and validation
        # More lenient confidence calculation
        max_expected_error = 2000.0  # Increased to be more reasonable
        normalized_error = min(error / max_expected_error, 1.0)
        confidence = 1.0 - normalized_error
        
        # Reduce confidence if validation failed (but not too severely)
        if not is_valid:
            confidence *= 0.6  # Less severe penalty for failed validation
        
        # Apply smoothing/filtering for stable estimates
        smoothed_position, smoothed_angle, smoothed_confidence = self._apply_smoothing(
            position, self.angle, confidence
        )
        
        # Update position estimate with smoothed values
        self.position = smoothed_position
        self.angle = smoothed_angle
        self.confidence = smoothed_confidence
        
        # If confidence is very low, consider resetting the localization system
        if smoothed_confidence < 0.1 and not is_valid:
            print("Very low confidence detected, considering localization reset...")
            # Don't reset immediately, but mark for potential reset
            if hasattr(self, '_consecutive_failures'):
                self._consecutive_failures += 1
            else:
                self._consecutive_failures = 1
            
            # Reset if we've had too many consecutive failures
            if self._consecutive_failures > 10:
                print("Too many consecutive failures, resetting localization system...")
                self.reset_localization()
                self._consecutive_failures = 0
        else:
            # Reset failure counter on successful localization
            if hasattr(self, '_consecutive_failures'):
                self._consecutive_failures = 0
        
        return smoothed_position.copy(), smoothed_confidence
    
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
    
    def get_confidence(self):
        """
        Get current localization confidence
        
        Returns:
            float: Confidence value (0-1)
        """
        return self.confidence
    
    def reset_position(self, x=0, y=0):
        """
        Reset position estimate to given coordinates
        
        Args:
            x: X coordinate in mm
            y: Y coordinate in mm
        """
        self.position = [x, y]
        self.confidence = 0.0
        self.best_guess = [x, y]
        self.best_error = float('inf')
        print(f"Position reset to ({x}, {y})")
    
    def reset_localization(self):
        """
        Reset the entire localization system
        This will force a global search on the next localization call
        """
        self.best_error = float('inf')
        self.confidence = 0.0
        print("Localization system reset - will perform global search next time")
    
    def get_sensor_data(self):
        """
        Get current sensor data for debugging
        
        Returns:
            dict: Sensor data including distances and angles
        """
        # Count valid measurements
        valid_measurements = 0
        for sensor in self.tof_manager.sensors:
            distance = self.tof_manager.sensor_distances[sensor.angle]
            if self.min_distance <= distance <= self.max_distance:
                valid_measurements += 1
        
        return {
            'tof_distances': self.tof_manager.get_all_distances(),
            'robot_angle': self.angle,
            'position': self.get_position(),
            'confidence': self.confidence,
            'sensor_count': self.tof_manager.get_sensor_count(),
            'valid_measurements': valid_measurements,
            'best_error': self.best_error if self.best_error != float('inf') else None,
            'consecutive_failures': getattr(self, '_consecutive_failures', 0),
            'grid_resolution': self.grid_resolution
        }
    
    def get_continuous_sensor_data(self):
        """
        Get continuously updated sensor data for real-time monitoring
        
        Returns:
            dict: Current sensor readings and robot state
        """
        # Update sensor readings
        self.tof_manager.update_distances()
        
        # Update robot angle
        current_angle = self._update_angle_from_imu()
        
        # Get current sensor readings
        sensor_readings = {}
        valid_sensors = 0
        
        for sensor in self.tof_manager.sensors:
            distance = self.tof_manager.sensor_distances[sensor.angle]
            sensor_readings[sensor.angle] = distance
            
            if self.min_distance <= distance <= self.max_distance:
                valid_sensors += 1
        
        return {
            'timestamp': time.time(),
            'robot_angle_deg': math.degrees(current_angle),
            'robot_angle_rad': current_angle,
            'sensor_readings': sensor_readings,
            'valid_sensors': valid_sensors,
            'total_sensors': len(self.tof_manager.sensors),
            'sensor_status': {
                'min_distance': self.min_distance,
                'max_distance': self.max_distance,
                'sensors_working': valid_sensors >= self.min_sensors
            }
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
        
        # Get expected readings for current position
        expected_readings = self._get_expected_readings(self.position, self.angle)
        
        # Calculate individual sensor errors
        sensor_errors = {}
        for sensor_angle in expected_readings:
            actual = actual_readings.get(sensor_angle, self.max_distance)
            expected = expected_readings[sensor_angle]
            if self.min_distance <= actual <= self.max_distance:
                sensor_errors[sensor_angle] = abs(actual - expected)
        
        return {
            'current_position': self.position.copy(),
            'current_angle_deg': math.degrees(self.angle),
            'confidence': self.confidence,
            'best_error': self.best_error if self.best_error != float('inf') else None,
            'actual_readings': actual_readings,
            'expected_readings': expected_readings,
            'sensor_errors': sensor_errors,
            'valid_measurements': sum(1 for d in actual_readings.values() 
                                   if self.min_distance <= d <= self.max_distance),
            'grid_info': {
                'resolution_mm': self.grid_resolution,
                'computation_method': 'on-the-fly ray casting'
            }
        }
    
    def is_localized(self, min_confidence=0.5):
        """
        Check if robot is well localized
        
        Args:
            min_confidence: Minimum confidence threshold
            
        Returns:
            bool: True if well localized
        """
        return self.confidence >= min_confidence
    
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
            if self.min_distance <= distance <= self.max_distance:
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
    
    def validate_localization(self, position, angle, tolerance=None):
        """
        Validate the localization result by checking consistency
        
        Args:
            position: [x, y] position in mm
            angle: Robot angle in radians
            tolerance: Maximum allowed error in mm (auto-scales with field size if None)
            
        Returns:
            bool: True if localization is valid
        """
        # Auto-scale tolerance with field size if not provided
        if tolerance is None:
            tolerance = max(500, 0.1 * min(self.field_width, self.field_height))
        # Check if position is within field bounds (with some margin)
        margin = 100  # Allow some margin outside field bounds
        if (position[0] < -margin or position[0] > self.field_width + margin or 
            position[1] < -margin or position[1] > self.field_height + margin):
            return False
        
        # Check error consistency (more lenient)
        error = self._compute_error(position, angle)
        if error > tolerance:
            return False
        
        # Check if we have enough valid sensors (reduced requirement)
        valid_measurements = 0
        for sensor in self.tof_manager.sensors:
            distance = self.tof_manager.sensor_distances[sensor.angle]
            if self.min_distance <= distance <= self.max_distance:
                valid_measurements += 1
        
        # Reduced minimum sensor requirement for better robustness
        return valid_measurements >= max(2, self.min_sensors - 1)
    
    def start_continuous_monitoring(self, update_interval=0.1):
        """
        Start continuous monitoring of robot angle and TOF sensor distances
        
        Args:
            update_interval: Time between updates in seconds
        """
        print("Starting continuous monitoring of robot state...")
        print(f"Update interval: {update_interval}s")
        
        try:
            while True:
                # Get continuous sensor data
                sensor_data = self.get_continuous_sensor_data()
                
                # Print current state
                print(f"[MONITOR] Time: {sensor_data['timestamp']:.1f}s, "
                      f"Angle: {sensor_data['robot_angle_deg']:.1f}°, "
                      f"Sensors: {sensor_data['valid_sensors']}/{sensor_data['total_sensors']}")
                
                # Print individual sensor readings
                for angle_rad, distance in sensor_data['sensor_readings'].items():
                    angle_deg = math.degrees(angle_rad)
                    status = "✓" if self.min_distance <= distance <= self.max_distance else "✗"
                    print(f"  Sensor {angle_deg:6.1f}°: {distance:4.0f}mm {status}")
                
                time.sleep(update_interval)
                
        except KeyboardInterrupt:
            print("\nStopping continuous monitoring...")
        except Exception as e:
            print(f"Error in continuous monitoring: {e}")
    
    def get_current_robot_state(self):
        """
        Get the current robot state with continuous updates
        
        Returns:
            dict: Current robot state including position, angle, and sensor data
        """
        # Update all sensor data
        sensor_data = self.get_continuous_sensor_data()
        
        return {
            'position': self.position.copy(),
            'angle_rad': self.angle,
            'angle_deg': math.degrees(self.angle),
            'confidence': self.confidence,
            'sensor_data': sensor_data,
            'localization_status': {
                'is_localized': self.is_localized(),
                'best_error': self.best_error if self.best_error != float('inf') else None,
                'consecutive_failures': getattr(self, '_consecutive_failures', 0)
            }
        }


# Test and demonstration code
if __name__ == "__main__":
    print("Testing real measurement localization system...")
    print("Available test modes:")
    print("1. Standard localization loop")
    print("2. Continuous monitoring mode")
    print("3. Single state check")
    
    # Initialize I2C bus
    import board
    import busio
    i2c = busio.I2C(board.SCL, board.SDA)
    
    # Create localization system
    localizer = Localizer(i2c_bus=i2c)
    
    # Check if user wants continuous monitoring
    try:
        choice = input("Enter test mode (1-3, default=1): ").strip()
        if choice == "2":
            print("Starting continuous monitoring mode...")
            localizer.start_continuous_monitoring(update_interval=0.5)
            exit(0)
        elif choice == "3":
            print("Single state check:")
            robot_state = localizer.get_current_robot_state()
            print(f"Position: {robot_state['position']}")
            print(f"Angle: {robot_state['angle_deg']:.1f}°")
            print(f"Valid sensors: {robot_state['sensor_data']['valid_sensors']}/{robot_state['sensor_data']['total_sensors']}")
            exit(0)
    except:
        # Default to standard mode if input fails
        pass
    
    if localizer.tof_manager.get_sensor_count() == 0:
        print("No TOF sensors available for localization!")
        exit(1)
    
    print(f"Localization system ready with {localizer.tof_manager.get_sensor_count()} TOF sensors")
    print("Starting continuous localization loop (press Ctrl+C to stop)...")
    
    # Sensor logging variables
    last_sensor_log_time = 0
    sensor_log_interval = 2.0  # Log closest sensor every 2 seconds
    
    # Localization loop with continuous updates
    try:
        while True:
            start_time = time.time()
            current_time = time.time()
            
            # Perform localization using real measurements with continuous updates
            position, confidence = localizer.localize()
            angle = localizer.get_angle()
            
            localization_time = time.time() - start_time
            
            # Print results with continuous monitoring info
            print(f"Position: ({position[0]:.1f}, {position[1]:.1f}) mm, "
                  f"Angle: {math.degrees(angle):.1f}°, "
                  f"Confidence: {confidence:.3f}, "
                  f"Error: {localizer.best_error:.1f}, "
                  f"Time: {localization_time*1000:.1f}ms")
            
            # Get continuous robot state
            robot_state = localizer.get_current_robot_state()
            print(f"[CONTINUOUS] Valid sensors: {robot_state['sensor_data']['valid_sensors']}/{robot_state['sensor_data']['total_sensors']}")
            
            # Log closest sensor every 2 seconds
            if current_time - last_sensor_log_time >= sensor_log_interval:
                closest_angle, closest_distance, closest_direction = localizer.get_closest_sensor()
                if closest_angle is not None:
                    print(f"[SENSOR LOG] Closest sensor: {closest_direction} "
                          f"(angle: {math.degrees(closest_angle):.1f}°, "
                          f"distance: {closest_distance:.1f}mm)")
                else:
                    print("[SENSOR LOG] No valid sensors detected")
                last_sensor_log_time = current_time
            
            # Wait before next localization
            time.sleep(LOCALIZATION_CONFIG["update_frequency"])
            
    except KeyboardInterrupt:
        print("\nStopping localization loop...")
    except Exception as e:
        print(f"Error in localization test: {e}")
        import traceback
        traceback.print_exc()
