"""
Configuration file for the Soccer Robot
Contains all major configuration parameters for easy modification
"""

# Camera Configuration
CAMERA_CONFIG = {
    "width": 640,
    "height": 480,
    "format": "RGB888"
}

# Ball Detection Configuration
BALL_DETECTION = {
    # HSV color range for orange ball detection
    "lower_orange": [0, 132, 61],
    "upper_orange": [14, 255, 255],
    
    # Contour filtering parameters
    "min_contour_area": 20,
    "max_contour_area": 30000,
    
    # Ball proximity and centering thresholds
    "proximity_threshold": 220,  # Ball area - ball is "close" when area is over this value
    "angle_tolerance": 0.3,  # Ball must be within 30% of frame center horizontally
}

# Circular Mask Configuration
CIRCULAR_MASK = {
    "center_x": 320,  # Center of 640x480 frame
    "center_y": 240,
    "radius": 110  # 20% bigger than 92 (92 * 1.20 = 110)
}

# Motor Configuration
MOTOR_CONFIG = {
    # Motor addresses for 4 omniwheels: [back left, back right, front left, front right]
    "addresses": [27, 28, 30, 26],
    
    # Speed parameters
    "max_speed": 15000000,  # Maximum speed for all movements
    "forward_speed": 0.9,  # Forward movement speed (70% of max)
    "turn_sensitivity": 1.2,  # Turn 1.2
    
    # Motor PID constants
    "current_limit_foc": 2 * 65536,
    "id_pid": {"kp": 1500, "ki": 200},
    "iq_pid": {"kp": 1500, "ki": 200},
    "speed_pid": {"kp": 4e-2, "ki": 4e-4, "kd": 3e-2},
    "position_pid": {"kp": 275, "ki": 0, "kd": 0},
    "position_region_boundary": 250000,
    
    # Calibration parameters
    "calibration_options": {
        "calibration_time": 300,
        "calibration_current": 2097152,
        "calibration_speed": 50000,
        "calibration_position": 500000
    },
    
    # Operating modes
    "operating_mode": 15,
    "sensor_mode": 1,
    "command_mode": 12,
    "final_operating_mode": 3,
    "final_command_mode": 12
}

# Frame Configuration
FRAME_CONFIG = {
    "center_x": 320,
    "center_y": 240
}

# IMU Configuration
IMU_CONFIG = {
    "enable_reports": [
        "BNO_REPORT_ACCELEROMETER",
        "BNO_REPORT_GYROSCOPE", 
        "BNO_REPORT_MAGNETOMETER",
        "BNO_REPORT_ROTATION_VECTOR"
    ]
}

# Control Loop Configuration
CONTROL_CONFIG = {
    "update_frequency": 0.001,  # Sleep time between control loop iterations
    "force_calibration": False  # Set to True to force motor calibration
}


