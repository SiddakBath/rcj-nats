"""
Motor Controller module for the Soccer Robot
Handles BLDC motor initialization, configuration, and control
"""

import time
import sys
from steelbar_powerful_bldc_driver import PowerfulBLDCDriver
from config import MOTOR_CONFIG


class MotorController:
    """Handles BLDC motor control for the soccer robot"""
    
    def __init__(self, i2c_bus):
        """
        Initialize motor controller
        
        Args:
            i2c_bus: I2C bus instance for motor communication
        """
        self.i2c = i2c_bus
        self.motors = []
        self.motor_modes = []
        
        # Motor parameters from config
        self.max_speed = MOTOR_CONFIG["max_speed"]
        self.forward_speed = MOTOR_CONFIG["forward_speed"]
        self.turn_sensitivity = MOTOR_CONFIG["turn_sensitivity"]
        
        # Initialize motors (will be called by main robot if needed)
        # self.setup_motors()  # Commented out to avoid double initialization
    
    def setup_motors(self, force_calibration=False):
        """
        Initialize and configure all motors
        
        Args:
            force_calibration: If True, force motor calibration
        """
        # Motor addresses from config
        motor_addresses = MOTOR_CONFIG["addresses"]
        
        for i, addr in enumerate(motor_addresses):
            motor = PowerfulBLDCDriver(self.i2c, addr)
            
            if motor.get_firmware_version() != 3:
                print(f"error: motor {i} firmware version {motor.get_firmware_version()}")
                continue
            
            # Set motor parameters from config
            motor.set_current_limit_foc(MOTOR_CONFIG["current_limit_foc"])
            motor.set_id_pid_constants(MOTOR_CONFIG["id_pid"]["kp"], MOTOR_CONFIG["id_pid"]["ki"])
            motor.set_iq_pid_constants(MOTOR_CONFIG["iq_pid"]["kp"], MOTOR_CONFIG["iq_pid"]["ki"])
            motor.set_speed_pid_constants(
                MOTOR_CONFIG["speed_pid"]["kp"], 
                MOTOR_CONFIG["speed_pid"]["ki"], 
                MOTOR_CONFIG["speed_pid"]["kd"]
            )
            motor.set_position_pid_constants(
                MOTOR_CONFIG["position_pid"]["kp"], 
                MOTOR_CONFIG["position_pid"]["ki"], 
                MOTOR_CONFIG["position_pid"]["kd"]
            )
            motor.set_position_region_boundary(MOTOR_CONFIG["position_region_boundary"])
            motor.set_speed_limit(self.max_speed)
            
            motor.configure_operating_mode_and_sensor(
                MOTOR_CONFIG["operating_mode"], 
                MOTOR_CONFIG["sensor_mode"]
            )
            motor.configure_command_mode(MOTOR_CONFIG["command_mode"])
            
            # Set calibration options from config
            calib_opts = MOTOR_CONFIG["calibration_options"]
            motor.set_calibration_options(
                calib_opts["calibration_time"],
                calib_opts["calibration_current"],
                calib_opts["calibration_speed"],
                calib_opts["calibration_position"]
            )
            
            if force_calibration:
                # Run full calibration
                motor.start_calibration()
                print(f"Starting calibration of motor {i}")
                while not motor.is_calibration_finished():
                    print(".", end="")
                    sys.stdout.flush()
                    time.sleep(0.001)
                print()
                print(f"  elecangleoffset: {motor.get_calibration_ELECANGLEOFFSET()}")
                print(f"  sincoscentre: {motor.get_calibration_SINCOSCENTRE()}")
            else:
                # Skip calibration - values are stored internally by the motor driver
                # The motor driver will use the previously calibrated values automatically
                print(f"Motor {i}: Using previously calibrated values (stored internally)")
                print(f"  elecangleoffset: {motor.get_calibration_ELECANGLEOFFSET()}")
                print(f"  sincoscentre: {motor.get_calibration_SINCOSCENTRE()}")
            
            motor.configure_operating_mode_and_sensor(
                MOTOR_CONFIG["final_operating_mode"], 
                MOTOR_CONFIG["sensor_mode"]
            )
            motor.configure_command_mode(MOTOR_CONFIG["final_command_mode"])
            
            self.motors.append(motor)
            self.motor_modes.append(12)
        
        print(f"initialized {len(self.motors)} motors")
    
    def set_motor_speeds(self, speeds):
        """
        Set speeds for all motors
        
        Args:
            speeds: List of speeds for each motor [back_left, back_right, front_left, front_right]
        """
        if len(self.motors) >= 4 and len(speeds) >= 4:
            for i, speed in enumerate(speeds):
                self.motors[i].set_speed(speed)
    
    def stop_motors(self):
        """Stop all motors"""
        for motor in self.motors:
            motor.set_speed(0)
    
    def update_motor_data(self):
        """Update motor data for all motors"""
        for motor in self.motors:
            motor.update_quick_data_readout()
    
    def get_motor_count(self):
        """Get the number of initialized motors"""
        return len(self.motors)
    
    def get_motor_speeds(self):
        """Get current motor speeds"""
        if len(self.motors) >= 4:
            return [motor.get_speed() for motor in self.motors]
        return [0, 0, 0, 0]
    
    def get_motor_status(self):
        """Get status information for all motors"""
        status = {
            'motor_count': len(self.motors),
            'speeds': self.get_motor_speeds(),
            'max_speed': self.max_speed,
            'forward_speed': self.forward_speed,
            'turn_sensitivity': self.turn_sensitivity
        }
        return status
