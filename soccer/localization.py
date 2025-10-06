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
        
        # Field configuration
        self.field_width = LOCALIZATION_CONFIG["field_width"]
        self.field_height = LOCALIZATION_CONFIG["field_height"]
        
        # Ray casting parameters
        self.min_sensors = 3  # Minimum sensors needed for localization
        self.max_distance = TOF_CONFIG["max_distance"]
        self.min_distance = TOF_CONFIG["min_distance"]
        
        # Optimization parameters
        self.best_guess = [0, 0]  # Best position estimate
        self.best_error = float('inf')  # Best error score
        
        # Field walls for ray casting
        self.walls = LOCALIZATION_CONFIG["walls"]
        
        print("Ray casting localization system initialized")
    
    def _cast_ray(self, position, angle):
        """
        Cast a ray from position at angle and find distance to closest wall
        
        Args:
            position: [x, y] position in mm
            angle: Ray angle in radians
            
        Returns:
            float: Distance to closest wall intersection
        """
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
        
        return math.sqrt(minimum_distance) if minimum_distance != float('inf') else self.max_distance
    
    def _cast_rays(self, position, bot_angle):
        """
        Cast rays for all TOF sensor angles from given position
        
        Args:
            position: [x, y] position in mm
            bot_angle: Robot heading angle in radians
            
        Returns:
            dict: Angle -> expected distance mapping
        """
        distances = {}
        for sensor in self.tof_manager.sensors:
            world_angle = bot_angle + sensor.angle
            dist = self._cast_ray(position, world_angle)
            distances[sensor.angle] = dist
        return distances
    
    def _compute_error(self, position, bot_angle):
        """
        Compute error between expected and actual TOF measurements
        
        Args:
            position: [x, y] position in mm
            bot_angle: Robot heading angle in radians
            
        Returns:
            float: Total error (sum of squared differences)
        """
        error = 0.0
        raycast_distances = self._cast_rays(position, bot_angle)
        
        for sensor in self.tof_manager.sensors:
            actual_distance = self.tof_manager.sensor_distances[sensor.angle]
            expected_distance = raycast_distances[sensor.angle]
            
            # Only include valid measurements in error calculation
            if self.min_distance <= actual_distance <= self.max_distance:
                diff = abs(expected_distance - actual_distance)
                error += diff * diff  # Squared difference
        
        return error
    
    def _optimize_position(self, bot_angle):
        """
        Optimize position using iterative grid search
        
        Args:
            bot_angle: Robot heading angle in radians
            
        Returns:
            tuple: (position, error) where position is [x, y] in mm
        """
        # Get optimization parameters from config
        move_amount = LOCALIZATION_CONFIG["initial_move_amount"]
        decay_rate = LOCALIZATION_CONFIG["decay_rate"]
        cutoff = LOCALIZATION_CONFIG["cutoff"]
        
        # Start with current best guess or field center
        if self.best_error == float('inf'):
            self.best_guess = [self.field_width / 2, self.field_height / 2]
            self.best_error = self._compute_error(self.best_guess, bot_angle)
        
        # Iterative grid search
        while move_amount > cutoff:
            converged = False
            while not converged:
                converged = True
                
                # Check 3x3 grid around current best guess
                for dx in range(-1, 2):
                    for dy in range(-1, 2):
                        guess_pos = [
                            self.best_guess[0] + move_amount * dx,
                            self.best_guess[1] + move_amount * dy
                        ]
                        
                        # Keep position within field bounds
                        guess_pos[0] = max(0, min(self.field_width, guess_pos[0]))
                        guess_pos[1] = max(0, min(self.field_height, guess_pos[1]))
                        
                        error = self._compute_error(guess_pos, bot_angle)
                        
                        if error < self.best_error:
                            converged = False
                            self.best_error = error
                            self.best_guess = guess_pos
            
            # Reduce step size
            move_amount *= decay_rate
        
        return self.best_guess.copy(), self.best_error
    
    
    def _update_angle_from_imu(self):
        """
        Update robot angle from IMU
        
        Returns:
            float: Robot angle in radians
        """
        heading_degrees = self.imu_sensor.get_compass_heading()
        if heading_degrees is not None:
            return math.radians(heading_degrees)
        return self.angle  # Keep current angle if IMU not available
    
    def localize(self):
        """
        Perform localization using ray casting algorithm with real TOF measurements
        
        Returns:
            tuple: (position, confidence) where position is [x, y] in mm
        """
        # Update TOF sensor readings with real sensor data
        self.tof_manager.update_distances()
        
        # Update robot angle from IMU
        self.angle = self._update_angle_from_imu()
        
        # Count valid measurements
        valid_measurements = 0
        for sensor in self.tof_manager.sensors:
            distance = self.tof_manager.sensor_distances[sensor.angle]
            if self.min_distance <= distance <= self.max_distance:
                valid_measurements += 1
        
        if valid_measurements < self.min_sensors:
            # Not enough valid measurements
            return self.position.copy(), 0.0
        
        # Optimize position using ray casting
        position, error = self._optimize_position(self.angle)
        
        # Calculate confidence based on error (lower error = higher confidence)
        # Normalize error to 0-1 range, then invert for confidence
        max_expected_error = 1000.0  # Adjust based on field size and sensor accuracy
        normalized_error = min(error / max_expected_error, 1.0)
        confidence = 1.0 - normalized_error
        
        # Update position estimate
        self.position = position
        self.confidence = confidence
        
        return position.copy(), confidence
    
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
        print(f"Position reset to ({x}, {y})")
    
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
            'best_error': self.best_error
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
                    if -22.5 <= angle_deg <= 22.5:
                        closest_direction = "Front"
                    elif 22.5 < angle_deg <= 67.5:
                        closest_direction = "Front-Left"
                    elif 67.5 < angle_deg <= 112.5:
                        closest_direction = "Left"
                    elif 112.5 < angle_deg <= 157.5:
                        closest_direction = "Back-Left"
                    elif 157.5 < angle_deg or angle_deg <= -157.5:
                        closest_direction = "Back"
                    elif -157.5 < angle_deg <= -112.5:
                        closest_direction = "Back-Right"
                    elif -112.5 < angle_deg <= -67.5:
                        closest_direction = "Right"
                    elif -67.5 < angle_deg <= -22.5:
                        closest_direction = "Front-Right"
        
        return closest_angle, closest_distance, closest_direction


# Test and demonstration code
if __name__ == "__main__":
    print("Testing real measurement localization system...")
    
    try:
        # Initialize I2C bus
        import board
        import busio
        i2c = busio.I2C(board.SCL, board.SDA)
        
        # Create localization system
        localizer = Localizer(i2c_bus=i2c)
        
        if localizer.tof_manager.get_sensor_count() == 0:
            print("No TOF sensors available for localization!")
            exit(1)
        
        print(f"Localization system ready with {localizer.tof_manager.get_sensor_count()} TOF sensors")
        print("Starting localization loop (press Ctrl+C to stop)...")
        
        # Sensor logging variables
        last_sensor_log_time = 0
        sensor_log_interval = 2.0  # Log closest sensor every 2 seconds
        
        # Localization loop
        while True:
            start_time = time.time()
            current_time = time.time()
            
            # Perform localization using real measurements
            position, confidence = localizer.localize()
            angle = localizer.get_angle()
            
            localization_time = time.time() - start_time
            
            # Print results
            print(f"Position: ({position[0]:.1f}, {position[1]:.1f}) mm, "
                  f"Angle: {math.degrees(angle):.1f}°, "
                  f"Confidence: {confidence:.3f}, "
                  f"Error: {localizer.best_error:.1f}, "
                  f"Time: {localization_time*1000:.1f}ms")
            
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
