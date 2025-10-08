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

# Goal Detection Configuration
GOAL_DETECTION = {
    # Blue goal (target goal) HSV color range
    "blue_goal": {
        "lower": [89, 75, 78],   # Lower HSV for blue
        "upper": [145, 255, 255], # Upper HSV for blue
        "min_contour_area": 50,  # Reduced for smaller goals
        "max_contour_area": 50000,
        "aspect_ratio_min": 0.3,  # Minimum aspect ratio for goal detection
        "aspect_ratio_max": 3.0, # Maximum aspect ratio for goal detection
    },
    
    # Yellow goal (opponent goal) HSV color range
    "yellow_goal": {
        "lower": [18, 179, 179],  # Lower HSV for yellow
        "upper": [65, 255, 255],  # Upper HSV for yellow
        "min_contour_area": 50,  # Reduced for smaller goals
        "max_contour_area": 50000,
        "aspect_ratio_min": 0.3,  # Minimum aspect ratio for goal detection
        "aspect_ratio_max": 3.0, # Maximum aspect ratio for goal detection
    },
    
    # Goal detection parameters
    "min_goal_width": 25,      # Minimum goal width in pixels (reduced for smaller goals)
    "min_goal_height": 15,     # Minimum goal height in pixels (reduced for smaller goals)
    "goal_center_tolerance": 0.4,  # Goal must be within 40% of frame center horizontally
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
    "max_speed": 35000000,  # Maximum speed for all movements
    "forward_speed": 0.9,  # Forward movement speed (70% of max)
    "turn_sensitivity": 1.2,  # Turn sensitivity for motor control
    
    # Turn smoothing parameters for sharper turns at right angles
    "turn_smoothing": {
        "edge_threshold": 0.3,  # When ball is 30% from center, apply edge multiplier
        "edge_multiplier": 2.0  # Additional multiplier for edge cases (ball at right angles)
    },
    
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

# TOF Sensor Configuration
TOF_CONFIG = {
    # TOF sensor addresses (I2C addresses) - real addresses from hardware
    "addresses": [0x53, 0x55, 0x50, 0x56, 0x5b, 0x57, 0x51, 0x54],
    
    # TOF sensor angles in degrees (relative to robot front) - matching physical positions
    "angles": [-90, 45, 0, -135, 180, -45, 90, 135],
    
    # TOF sensor offsets (x, y) in mm from robot center
    # Each sensor is offset 10 units in the direction it's pointing
    "offsets": {
        0x53: (0, 10),      # Left (-90°) - pointing left, so offset in +Y direction
        0x55: (7, -7),      # Front-left (45°) - pointing front-left
        0x50: (10, 0),      # Front (0°) - pointing front, so offset in +X direction
        0x56: (-7, -7),     # Back-right (-135°) - pointing back-right
        0x5b: (-10, 0),     # Back (180°) - pointing back, so offset in -X direction
        0x57: (7, 7),       # Front-right (-45°) - pointing front-right
        0x51: (0, -10),     # Right (90°) - pointing right, so offset in -Y direction
        0x54: (-7, 7)       # Back-left (135°) - pointing back-left
    },
    
    # Distance limits
    "max_distance": 1000,  # Maximum distance in mm
    "min_distance": 10,    # Minimum distance in mm
    "read_register": 0x10  # Register to read distance from
}

# Localization Configuration
LOCALIZATION_CONFIG = {
    # Field dimensions in mm
    "field_width": 2430,   # 2.43 meters
    "field_height": 1820,  # 1.82 meters
    
    # Field walls configuration (coordinates match robot's coordinate system: 0,0 at field corner)
    "walls": [
        # field walls (robot coordinate system: 0,0 at bottom-left corner)
        {'type': 'vertical',   'x': 0,       'y_min': 0,      'y_max': 1820}, # left wall
        {'type': 'vertical',   'x': 2430,    'y_min': 0,      'y_max': 1820}, # right wall  
        {'type': 'horizontal', 'y': 1820,    'x_min': 0,      'x_max': 2430}, # top wall
        {'type': 'horizontal', 'y': 0,        'x_min': 0,      'x_max': 2430}, # bottom wall
        # goal walls (adjusted for robot coordinate system)
        {'type': 'vertical',   'x': 989,     'y_min': 685,    'y_max': 1135},  # right goal back
        {'type': 'vertical',   'x': 1441,    'y_min': 685,    'y_max': 1135},  # left goal back  
        {'type': 'horizontal', 'y': 1135,    'x_min': 915,    'x_max': 2430}, # right goal top side wall
        {'type': 'horizontal', 'y': 685,     'x_min': 915,    'x_max': 2430}, # right goal bottom side wall
        {'type': 'horizontal', 'y': 1135,    'x_min': 0,      'x_max': 915}, # left goal top side wall
        {'type': 'horizontal', 'y': 685,     'x_min': 0,      'x_max': 915}, # left goal bottom side wall
    ],
    
    # Localization update frequency
    "update_frequency": 0.1       # Localization update frequency in seconds
}


