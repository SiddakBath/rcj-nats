"""
Soccer Robot Package
A modular soccer robot system with camera, motor control, IMU, and control systems
"""

from .robot import SoccerRobot
from .camera import Camera
from .motor_controller import MotorController
from .imu_sensor import IMUSensor
from .control_system import ControlSystem
from . import config

__version__ = "1.0.0"
__author__ = "Soccer Robot Team"

__all__ = [
    'SoccerRobot',
    'Camera', 
    'MotorController',
    'IMUSensor',
    'ControlSystem',
    'config'
]
