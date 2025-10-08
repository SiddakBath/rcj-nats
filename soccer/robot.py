"""
Main Robot class for the Soccer Robot
Orchestrates all modules and provides the main control loop
"""

import time
import board
import busio
from camera import Camera
from motor_controller import MotorController
from imu_sensor import IMUSensor
from control_system import ControlSystem
from config import CONTROL_CONFIG


class SoccerRobot:
    """Main soccer robot class that orchestrates all subsystems"""
    
    def __init__(self, force_calibration=False):
        """
        Initialize the soccer robot with all subsystems
        
        Args:
            force_calibration: If True, force motor calibration
        """
        print("Initializing Soccer Robot...")
        
        # Initialize I2C bus
        self.i2c = busio.I2C(board.SCL, board.SDA)
        
        # Initialize all subsystems
        self.camera = Camera()
        self.motor_controller = MotorController(self.i2c)
        self.motor_controller.setup_motors()  # Initialize motors
        self.imu_sensor = IMUSensor(self.i2c)
        self.control_system = ControlSystem()
        
        # Force calibration if requested
        if force_calibration:
            print("WARNING: Force calibration is enabled!")
            print("This will recalibrate all motors and may take several minutes.")
            print("Only do this if motors aren't working properly or after hardware changes.")
            print("Starting calibration automatically...")
            self.motor_controller.setup_motors(force_calibration=True)
        
        # Initialize IMU relative heading
        if self.imu_sensor.is_available():
            self.imu_sensor.initialize_relative_heading()
        
        print("Soccer Robot initialization complete!")
    
    def run(self):
        """Main control loop for the soccer robot"""
        print("Soccer robot starting...")
        print("Press Ctrl+C to quit")
        
        try:
            while True:
                # Capture frame from camera
                frame = self.camera.capture_frame()
                
                # Detect ball in frame
                ball_found, ball_center, ball_radius = self.camera.detect_ball(frame)
                
                if ball_found:
                    # Move forward and turn towards ball when detected
                    speeds = self.control_system.calculate_motor_commands(
                        ball_found, ball_center[0], ball_center[1]
                    )
                    self.motor_controller.set_motor_speeds(speeds)
                    
                    # Get control information for display
                    control_info = self.control_system.get_control_info(
                        ball_found, ball_center[0], ball_center[1]
                    )
                    
                    # Get ball proximity information
                    proximity_info = self.camera.get_ball_proximity_info()
                    
                    # Get orientation information
                    orientation_info = self.imu_sensor.get_orientation_info()
                    
                    # Format display information
                    close_prefix = "CLOSE+CENTERED - " if proximity_info['is_close_and_centered'] else "CLOSE - " if proximity_info['is_close'] else ""
                    
                    print(f"{close_prefix}Ball at ({ball_center[0]}, {ball_center[1]}) - "
                          f"Area: {proximity_info['ball_area']:.1f}px² - "
                          f"H_Error: {proximity_info['horizontal_error']:.3f} "
                          f"V_Error: {proximity_info['vertical_error']:.3f} - "
                          f"{control_info['turn_mode']} - "
                          f"Turn: {control_info['turn_adjustment']//1000}k - "
                          f"Speeds: BL:{speeds[0]//1000}k BR:{speeds[1]//1000}k "
                          f"FL:{speeds[2]//1000}k FR:{speeds[3]//1000}k - "
                          f"{orientation_info['heading_str']} ({orientation_info['relative_str']})")
                else:
                    # Stop motors when ball not detected
                    self.motor_controller.stop_motors()
                    
                    # Get orientation information even when ball not found
                    orientation_info = self.imu_sensor.get_orientation_info()
                    
                    print(f"Ball not found - motors stopped - "
                          f"{orientation_info['heading_str']} ({orientation_info['relative_str']})")
                
                # Update motor data
                self.motor_controller.update_motor_data()
                
                # Control loop frequency
                time.sleep(CONTROL_CONFIG["update_frequency"])
                
        except KeyboardInterrupt:
            print("Shutting down...")
        finally:
            self.shutdown()
    
    def shutdown(self):
        """Shutdown the robot and all subsystems"""
        print("Stopping motors...")
        self.motor_controller.stop_motors()
        
        print("Stopping camera...")
        self.camera.stop()
        
        print("Robot shutdown complete")
    
    def get_status(self):
        """Get comprehensive status of all subsystems"""
        status = {
            'camera': {
                'ball_detected': self.camera.ball_detected,
                'ball_position': self.camera.get_ball_position(),
                'proximity_info': self.camera.get_ball_proximity_info()
            },
            'motors': self.motor_controller.get_motor_status(),
            'imu': {
                'available': self.imu_sensor.is_available(),
                'orientation': self.imu_sensor.get_orientation_info()
            },
            'control': self.control_system.get_control_parameters()
        }
        return status
    
    def reset_imu_heading(self):
        """Reset the IMU initial heading to current heading"""
        self.imu_sensor.reset_initial_heading()
    
    def get_imu_data(self):
        """Get comprehensive IMU data"""
        return self.imu_sensor.get_imu_data()
    
    def get_ball_info(self):
        """Get ball detection information"""
        return {
            'position': self.camera.get_ball_position(),
            'proximity': self.camera.get_ball_proximity_info()
        }
    


def main():
    """Main entry point for the soccer robot"""
    # Get force_calibration setting from config
    force_calibration = CONTROL_CONFIG["force_calibration"]
    
    # Create and run the robot
    robot = SoccerRobot(force_calibration=force_calibration)
    robot.run()


if __name__ == "__main__":
    main()
