# Soccer Robot Localization System Implementation

## Overview
This document describes the implementation of a comprehensive localization system for the soccer robot using 8 TOF (Time-of-Flight) sensors and IMU data for triangulation-based position estimation.

## Key Features

### 1. **Triangulation Algorithm**
- **8 TOF Sensors**: Positioned around the robot at angles: -90°, 45°, 0°, -135°, 180°, -45°, 90°, 135°
- **Ray Casting**: Calculates expected distances to field walls for each sensor
- **Trilateration**: Uses geometric trilateration to solve for robot position from multiple sensor readings
- **Error Minimization**: Tests multiple sensor combinations to find the most accurate position

### 2. **Field Configuration**
- **Field Dimensions**: 2430mm x 1820mm (2.43m x 1.82m)
- **Wall System**: 10 walls including field boundaries and goal areas
- **Coordinate System**: Robot coordinate system with (0,0) at bottom-left corner

### 3. **Sensor Management**
- **TOF Sensor Manager**: Handles 8 VL53L0X sensors with individual addressing
- **Distance Validation**: Filters readings within 10mm-1000mm range
- **Health Monitoring**: Tracks sensor status and reliability

### 4. **IMU Integration**
- **BNO085 IMU**: Provides compass heading and relative orientation
- **Angle Tracking**: Maintains robot orientation relative to initial heading
- **Heading Reset**: Allows recalibration of initial heading reference

## Core Algorithms

### Ray Casting to Walls
```python
def _ray_cast_to_walls(self, robot_x, robot_y, sensor_angle):
    """
    Cast a ray from robot position at given angle to find intersection with field walls
    """
    # Calculate absolute sensor direction (robot angle + sensor angle)
    absolute_angle = self.angle + sensor_angle
    
    # Ray direction vector
    dx = math.cos(absolute_angle)
    dy = math.sin(absolute_angle)
    
    # Check intersection with each wall (vertical and horizontal)
    # Return minimum distance to wall intersection
```

### Trilateration Algorithm
```python
def _trilaterate_position(self, x1, y1, r1, x2, y2, r2, x3, y3, r3):
    """
    Trilateration algorithm to find position from 3 distance measurements
    Uses geometric solution for system of equations:
    (x - x1)² + (y - y1)² = r1²
    (x - x2)² + (y - y2)² = r2²  
    (x - x3)² + (y - y3)² = r3²
    """
```

### Position Triangulation
```python
def _triangulate_position(self):
    """
    Main triangulation method that:
    1. Collects valid sensor readings
    2. Tests all combinations of 3 sensors
    3. Solves for position using trilateration
    4. Selects solution with minimum error
    """
```

## Key Methods

### Core Localization
- `localize()`: Main localization method that updates sensors and performs triangulation
- `get_position()`: Returns current position estimate [x, y] in mm
- `get_angle()`: Returns current angle estimate in radians

### Sensor Management
- `get_sensor_data()`: Returns all sensor readings and robot state
- `get_sensor_health_status()`: Monitors sensor health and reliability
- `get_closest_sensor()`: Finds sensor with shortest distance reading

### Accuracy and Validation
- `get_localization_accuracy()`: Calculates accuracy level and error estimates
- `get_field_bounds_check()`: Validates position is within field boundaries
- `_validate_sensor_reading()`: Compares measured vs expected distances

### Testing and Diagnostics
- `test_localization_system()`: Comprehensive system test with known positions
- `get_localization_debug_info()`: Detailed debugging information
- `get_imu_status()`: IMU sensor status and heading information

## Configuration

### TOF Sensor Configuration
```python
TOF_CONFIG = {
    "addresses": [0x53, 0x55, 0x50, 0x56, 0x5b, 0x57, 0x51, 0x54],
    "angles": [-90, 45, 0, -135, 180, -45, 90, 135],
    "offsets": {
        0x53: (0, 10),      # Left sensor
        0x55: (7, -7),      # Front-left sensor
        0x50: (10, 0),      # Front sensor
        0x56: (-7, -7),     # Back-right sensor
        0x5b: (-10, 0),     # Back sensor
        0x57: (7, 7),       # Front-right sensor
        0x51: (0, -10),     # Right sensor
        0x54: (-7, 7)       # Back-left sensor
    },
    "max_distance": 1000,  # Maximum distance in mm
    "min_distance": 10,    # Minimum distance in mm
}
```

### Field Configuration
```python
LOCALIZATION_CONFIG = {
    "field_width": 2430,   # 2.43 meters
    "field_height": 1820,  # 1.82 meters
    "walls": [
        # Field boundaries and goal walls
        {'type': 'vertical', 'x': 0, 'y_min': 0, 'y_max': 1820},      # Left wall
        {'type': 'vertical', 'x': 2430, 'y_min': 0, 'y_max': 1820},   # Right wall
        {'type': 'horizontal', 'y': 1820, 'x_min': 0, 'x_max': 2430},  # Top wall
        {'type': 'horizontal', 'y': 0, 'x_min': 0, 'x_max': 2430},     # Bottom wall
        # Goal walls...
    ]
}
```

## Usage Example

```python
from soccer.localization import Localizer

# Initialize localization system
localizer = Localizer()

# Perform localization
position = localizer.localize()
print(f"Robot position: ({position[0]:.1f}, {position[1]:.1f}) mm")

# Get accuracy information
accuracy = localizer.get_localization_accuracy()
print(f"Localization accuracy: {accuracy['accuracy']}")

# Check sensor health
health = localizer.get_sensor_health_status()
print(f"Sensor health: {health['health_percentage']:.1f}%")
```

## Error Handling

### Robust Error Management
- **Sensor Validation**: Filters out-of-range readings
- **Fallback Mechanisms**: Uses last known position if triangulation fails
- **Error Tolerance**: 50mm tolerance for trilateration validation
- **Health Monitoring**: Tracks sensor reliability and system status

### Accuracy Levels
- **Excellent**: < 50mm average error
- **Good**: 50-100mm average error  
- **Fair**: 100-200mm average error
- **Poor**: > 200mm average error

## Testing

The system includes comprehensive testing capabilities:
- **Ray Casting Tests**: Validates wall intersection calculations
- **Position Tests**: Tests known positions for accuracy
- **Sensor Health Tests**: Monitors sensor reliability
- **Bounds Checking**: Ensures position is within field boundaries

## Performance Characteristics

- **Update Rate**: Configurable (default 10Hz)
- **Accuracy**: Sub-100mm typical accuracy with good sensor coverage
- **Reliability**: Degrades gracefully with sensor failures
- **Computational**: Efficient O(n³) for sensor combinations

## Dependencies

- **Hardware**: 8x VL53L0X TOF sensors, BNO085 IMU
- **Software**: Python 3.x, math, itertools
- **Configuration**: Field dimensions, sensor positions, wall definitions

This implementation provides a robust, accurate localization system suitable for autonomous soccer robot navigation.
