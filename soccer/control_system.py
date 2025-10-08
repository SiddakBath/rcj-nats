"""
Control System module for the Soccer Robot
Handles motor command calculations and control logic
"""

import numpy as np
from config import MOTOR_CONFIG, FRAME_CONFIG


class ControlSystem:
    """Handles motor command calculations and control logic"""
    
    def __init__(self):
        """Initialize control system with parameters from config"""
        # Speed parameters from config
        self.max_speed = MOTOR_CONFIG["max_speed"]
        self.forward_speed = MOTOR_CONFIG["forward_speed"]
        self.turn_sensitivity = MOTOR_CONFIG["turn_sensitivity"]
        
        # Frame configuration
        self.frame_center_x = FRAME_CONFIG["center_x"]
        self.frame_center_y = FRAME_CONFIG["center_y"]
    
    def calculate_turn_adjustment(self, error_x_norm):
        """
        Calculate turn adjustment using a non-linear smoothing function.
        
        This function provides:
        - Sharper turns when the ball is at right angles (high error values)
        - Smooth control when the ball is close to center (low error values)
        - Maximum turning force when the ball is at the edge of the frame
        
        Args:
            error_x_norm: Normalized horizontal error (-1.0 to 1.0)
            
        Returns:
            float: Turn adjustment value for motor control
        """
        # Clamp error to valid range
        error_x_norm = np.clip(error_x_norm, -1.0, 1.0)
        
        # Get smoothing parameters from config
        smoothing_config = MOTOR_CONFIG["turn_smoothing"]
        edge_threshold = smoothing_config["edge_threshold"]
        edge_multiplier = smoothing_config["edge_multiplier"]
        
        # Use a non-linear function that provides sharper turns for larger errors
        # For small errors (ball near center): gentle response
        # For large errors (ball at right angles): sharp response
        
        # Apply a sigmoid-like function that amplifies larger errors
        # Use absolute value for calculation, then restore sign
        abs_error = abs(error_x_norm)
        
        if abs_error < 0.1:
            # Very gentle response for small errors (ball near center)
            smoothed_error = error_x_norm * 0.5
        elif abs_error < 0.3:
            # Moderate response for medium errors
            smoothed_error = error_x_norm * 0.8
        else:
            # Sharp response for large errors (ball at right angles)
            # Use a function that amplifies the error
            smoothed_error = error_x_norm * (1.0 + abs_error * 2.0)
        
        # Apply additional sharpening for extreme angles (ball at edges)
        # This ensures maximum turning force when ball is at right angles
        if abs_error > edge_threshold:
            # Apply additional multiplier for edge cases
            smoothed_error *= edge_multiplier
        
        # Scale by max speed and turn sensitivity
        turn_adjustment = smoothed_error * self.max_speed * self.turn_sensitivity
        
        return turn_adjustment
    
    def calculate_motor_commands(self, ball_detected, ball_center_x, ball_center_y):
        """
        Calculate motor commands based on ball position
        
        Args:
            ball_detected: Whether ball is detected
            ball_center_x: X coordinate of ball center
            ball_center_y: Y coordinate of ball center
            
        Returns:
            list: Motor speeds [back_left, back_right, front_left, front_right]
        """
        # Calculate error from ball position to center of frame
        if ball_detected:
            error_x = ball_center_x - self.frame_center_x
            error_x_norm = error_x / self.frame_center_x
        else:
            # Default values when ball not detected
            error_x_norm = 0
        
        # Apply non-linear smoothing function for sharper turns at right angles
        turn_adjustment = self.calculate_turn_adjustment(error_x_norm)
        
        # Base forward movement speed
        forward_speed = self.max_speed * self.forward_speed
        
        # Calculate individual motor speeds with turning
        # Motors: [27-back left, 28-back right, 30-front left, 26-front right]
        # For omniwheels, turning is achieved by differential speed between left and right sides
        # To turn right (ball on left): speed up left motors, slow down right motors
        # To turn left (ball on right): speed up right motors, slow down left motors
        
        # Calculate left and right side speeds
        # Positive turn_adjustment means turn right (ball on left), negative means turn left (ball on right)
        left_side_speed = forward_speed + turn_adjustment
        right_side_speed = forward_speed - turn_adjustment
        
        # Back-left motor (27): left side speed (INVERTED)
        back_left_speed = -left_side_speed
        
        # Back-right motor (28): right side speed
        back_right_speed = right_side_speed
        
        # Front-left motor (30): right side speed (INVERTED) - matches omniwheel test
        front_left_speed = -right_side_speed
        
        # Front-right motor (26): right side speed
        front_right_speed = right_side_speed
        
        # Clip speeds to max limits
        speeds = [back_left_speed, back_right_speed, front_left_speed, front_right_speed]
        speeds = [np.clip(speed, -self.max_speed, self.max_speed) for speed in speeds]
        
        return [int(speed) for speed in speeds]
    
    def get_control_info(self, ball_detected, ball_center_x, ball_center_y):
        """
        Get control information for display and debugging
        
        Args:
            ball_detected: Whether ball is detected
            ball_center_x: X coordinate of ball center
            ball_center_y: Y coordinate of ball center
            
        Returns:
            dict: Control information including errors, adjustments, and mode
        """
        if ball_detected:
            error_x = ball_center_x - self.frame_center_x
            error_x_norm = error_x / self.frame_center_x
        else:
            error_x_norm = 0
        
        turn_adjustment = self.calculate_turn_adjustment(error_x_norm)
        turn_mode = "TURNING" if abs(turn_adjustment) > 1000 else "STRAIGHT"
        direction = "LEFT" if error_x_norm < 0 else "RIGHT" if error_x_norm > 0 else "CENTER"
        
        return {
            'error_x_norm': error_x_norm,
            'turn_adjustment': turn_adjustment,
            'turn_mode': turn_mode,
            'direction': direction,
            'forward_speed': self.max_speed * self.forward_speed
        }
    
    
    def get_control_parameters(self):
        """Get current control parameters"""
        return {
            'max_speed': self.max_speed,
            'forward_speed': self.forward_speed,
            'turn_sensitivity': self.turn_sensitivity,
            'frame_center_x': self.frame_center_x,
            'frame_center_y': self.frame_center_y
        }
    
    def test_turning_logic(self, error_x_norm):
        """
        Test method to verify turning logic calculations
        
        Args:
            error_x_norm: Normalized horizontal error (-1.0 to 1.0)
            
        Returns:
            dict: Detailed calculation breakdown for debugging
        """
        turn_adjustment = self.calculate_turn_adjustment(error_x_norm)
        forward_speed = self.max_speed * self.forward_speed
        
        left_side_speed = forward_speed + turn_adjustment
        right_side_speed = forward_speed - turn_adjustment
        
        return {
            'error_x_norm': error_x_norm,
            'turn_adjustment': turn_adjustment,
            'forward_speed': forward_speed,
            'left_side_speed': left_side_speed,
            'right_side_speed': right_side_speed,
            'back_left_speed': -left_side_speed,
            'back_right_speed': right_side_speed,
            'front_left_speed': -right_side_speed,
            'front_right_speed': right_side_speed
        }
