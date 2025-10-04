# Soccer Robot - Modular Architecture

This directory contains a modular soccer robot system with clean separation of concerns and well-organized components.

## Architecture Overview

The soccer robot has been refactored into a clean, modular architecture with the following components:

### Core Modules

- **`robot.py`** - Main robot class that orchestrates all subsystems
- **`camera.py`** - Camera handling and ball detection
- **`motor_controller.py`** - BLDC motor control and management
- **`imu_sensor.py`** - IMU sensor for compass and orientation data
- **`control_system.py`** - Motor command calculations and control logic
- **`config.py`** - Configuration parameters for all modules
- **`__init__.py`** - Package initialization and exports

## Module Descriptions

### Camera Module (`camera.py`)
- Handles Pi Camera initialization and frame capture
- Implements orange ball detection using HSV color filtering
- Applies circular mask to ignore center area
- Provides ball proximity and centering information
- Manages ball detection state and position tracking

### Motor Controller Module (`motor_controller.py`)
- Manages BLDC motor initialization and configuration
- Handles motor calibration (with optional force calibration)
- Provides motor speed control and status monitoring
- Manages motor data updates and communication

### IMU Sensor Module (`imu_sensor.py`)
- Interfaces with BNO085 IMU sensor for compass functionality
- Provides compass heading and relative heading calculations
- Offers comprehensive IMU data (accelerometer, gyroscope, magnetometer)
- Handles IMU initialization and error management

### Control System Module (`control_system.py`)
- Implements motor command calculations based on ball position
- Provides non-linear turn adjustment for sharper turns at right angles
- Calculates individual motor speeds for omniwheel movement
- Offers control information for debugging and display

### Main Robot Class (`robot.py`)
- Orchestrates all subsystems and provides main control loop
- Handles robot initialization and shutdown
- Manages the main control loop with ball detection and motor control
- Provides comprehensive status information and debugging capabilities

## Usage

### Basic Usage
```python
from soccer import SoccerRobot

# Create robot instance
robot = SoccerRobot()

# Run the robot
robot.run()
```

### With Force Calibration
```python
from soccer import SoccerRobot

# Create robot with force calibration
robot = SoccerRobot(force_calibration=True)
robot.run()
```

### Accessing Individual Modules
```python
from soccer import Camera, MotorController, IMUSensor, ControlSystem
import board
import busio

# Initialize I2C bus
i2c = busio.I2C(board.SCL, board.SDA)

# Create individual modules
camera = Camera()
motors = MotorController(i2c)
imu = IMUSensor(i2c)
control = ControlSystem()
```

## Configuration

All configuration parameters are centralized in `config.py`:

- **Camera Configuration**: Resolution, format, detection parameters
- **Motor Configuration**: Addresses, speeds, PID constants, calibration
- **IMU Configuration**: Sensor reports and settings
- **Control Configuration**: Update frequency, calibration settings

## Key Features

- **Modular Design**: Clean separation of concerns with focused modules
- **Configurable**: All parameters centralized in config file
- **Extensible**: Easy to add new features or modify existing ones
- **Maintainable**: Clear interfaces and well-documented code
- **Testable**: Individual modules can be tested independently
- **Robust**: Proper error handling and graceful shutdown

## Dependencies

- OpenCV (`cv2`) for computer vision
- NumPy for numerical operations
- PiCamera2 for camera interface
- Adafruit BNO08x for IMU sensor
- Steelbar BLDC driver for motor control
- Board and BusIO for I2C communication

## File Structure

```
soccer/
├── __init__.py          # Package initialization
├── robot.py             # Main robot class
├── camera.py            # Camera and ball detection
├── motor_controller.py  # Motor control
├── imu_sensor.py        # IMU sensor interface
├── control_system.py    # Control logic
├── config.py            # Configuration
└── README.md            # This file
```

## Migration from Monolithic Structure

The original `soccer_robot.py` file has been removed and replaced with this modular structure. The functionality remains the same, but the code is now:

- More maintainable
- Easier to test
- More extensible
- Better organized
- More professional

All original functionality has been preserved while improving code organization and maintainability.
