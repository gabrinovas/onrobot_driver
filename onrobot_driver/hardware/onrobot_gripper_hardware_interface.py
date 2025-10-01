#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from threading import Lock

# Correct ROS2 Control imports for Humble
from hardware_interface import SystemInterface, CallbackReturn, HardwareInfo
from hardware_interface.types import LIFE_CYCLE_STATE
from hardware_interface.hardware_info import HardwareInfo

from onrobot_driver.drivers.onrobot_gripper import OnRobotGripper

class OnRobotGripperHardwareInterface(SystemInterface):
    """
    ROS2 Control Hardware Interface for OnRobot Grippers
    """
    
    def __init__(self):
        super().__init__()
        self.node_ = None
        self.gripper_ = None
        self.lock_ = Lock()
        
        # Gripper state
        self.position_ = 0.0
        self.velocity_ = 0.0
        self.effort_ = 0.0
        self.command_position_ = 0.0
        self.last_command_position_ = 0.0
        
        # Configuration
        self.ip_address_ = "192.168.1.1"
        self.port_ = 502
        self.gripper_type_ = "2FG7"
        self.max_width_ = 0.085
        self.min_width_ = 0.0
        self.max_force_ = 100.0
        
    def on_init(self, hardware_info: HardwareInfo):
        """
        Initialize the hardware interface
        """
        try:
            # Get parameters from hardware info
            self.ip_address_ = hardware_info.hardware_parameters.get('ip_address', '192.168.1.1')
            self.port_ = int(hardware_info.hardware_parameters.get('port', '502'))
            self.gripper_type_ = hardware_info.hardware_parameters.get('gripper_type', '2FG7')
            self.max_width_ = float(hardware_info.hardware_parameters.get('max_width', '0.085'))
            self.min_width_ = float(hardware_info.hardware_parameters.get('min_width', '0.0'))
            self.max_force_ = float(hardware_info.hardware_parameters.get('max_force', '100.0'))
            
            print(f"OnRobot Gripper Hardware Interface initialized: "
                  f"IP: {self.ip_address_}, Type: {self.gripper_type_}")
            
            return CallbackReturn.SUCCESS
            
        except Exception as e:
            print(f"Error initializing hardware interface: {e}")
            return CallbackReturn.ERROR
    
    def on_configure(self, previous_state: LIFE_CYCLE_STATE):
        """
        Configure the hardware interface
        """
        try:
            # Initialize ROS node for the gripper driver
            if not rclpy.ok():
                rclpy.init()
            
            self.node_ = Node('onrobot_gripper_hardware_interface')
            
            # Initialize the gripper driver
            self.gripper_ = OnRobotGripper(self.node_)
            
            print("Configured OnRobot Gripper Hardware Interface")
            return CallbackReturn.SUCCESS
        except Exception as e:
            print(f"Error configuring hardware interface: {e}")
            return CallbackReturn.ERROR
    
    def on_activate(self, previous_state: LIFE_CYCLE_STATE):
        """
        Activate the hardware interface
        """
        try:
            print("Activating OnRobot Gripper Hardware Interface")
            return CallbackReturn.SUCCESS
        except Exception as e:
            print(f"Error activating hardware interface: {e}")
            return CallbackReturn.ERROR
    
    def on_deactivate(self, previous_state: LIFE_CYCLE_STATE):
        """
        Deactivate the hardware interface
        """
        try:
            print("Deactivating OnRobot Gripper Hardware Interface")
            return CallbackReturn.SUCCESS
        except Exception as e:
            print(f"Error deactivating hardware interface: {e}")
            return CallbackReturn.ERROR
    
    def on_cleanup(self):
        """
        Cleanup the hardware interface
        """
        try:
            if self.node_:
                self.node_.destroy_node()
            rclpy.shutdown()
            return CallbackReturn.SUCCESS
        except Exception as e:
            print(f"Error cleaning up hardware interface: {e}")
            return CallbackReturn.ERROR
    
    def on_shutdown(self):
        """
        Shutdown the hardware interface
        """
        try:
            return CallbackReturn.SUCCESS
        except Exception as e:
            print(f"Error shutting down hardware interface: {e}")
            return CallbackReturn.ERROR
    
    def on_error(self, previous_state: LIFE_CYCLE_STATE):
        """
        Handle errors
        """
        print("OnRobot Gripper Hardware Interface entered error state")
        return CallbackReturn.SUCCESS
    
    def export_state_interfaces(self):
        """
        Export state interfaces for ROS2 Control
        """
        interfaces = []
        
        # Add state interfaces for both finger joints
        interfaces.append(('left_finger_joint', 'position', self.position_))
        interfaces.append(('left_finger_joint', 'velocity', self.velocity_))
        interfaces.append(('left_finger_joint', 'effort', self.effort_))
        
        interfaces.append(('right_finger_joint', 'position', self.position_))  # Mirrored
        interfaces.append(('right_finger_joint', 'velocity', self.velocity_))
        interfaces.append(('right_finger_joint', 'effort', self.effort_))
        
        return interfaces
    
    def export_command_interfaces(self):
        """
        Export command interfaces for ROS2 Control
        """
        interfaces = []
        
        # Add command interface for position control
        interfaces.append(('left_finger_joint', 'position', self.command_position_))
        
        return interfaces
    
    def read(self):
        """
        Read data from hardware
        """
        try:
            with self.lock_:
                if self.gripper_:
                    # Read actual position from gripper
                    self.position_ = self.gripper_.current_position
                    # For simulation, you might want to get velocity and effort from the driver
                    self.velocity_ = 0.0  # Could be computed from position changes
                    self.effort_ = self.gripper_.current_force
            
            return CallbackReturn.SUCCESS
            
        except Exception as e:
            print(f"Error reading from hardware: {e}")
            return CallbackReturn.ERROR
    
    def write(self):
        """
        Write commands to hardware
        """
        try:
            with self.lock_:
                # Only send command if position has changed significantly
                if (self.gripper_ and 
                    abs(self.command_position_ - self.last_command_position_) > 0.001):
                    
                    success = self.gripper_.move_to_position(
                        self.command_position_, 
                        self.max_force_
                    )
                    
                    if success:
                        self.last_command_position_ = self.command_position_
            
            return CallbackReturn.SUCCESS
            
        except Exception as e:
            print(f"Error writing to hardware: {e}")
            return CallbackReturn.ERROR