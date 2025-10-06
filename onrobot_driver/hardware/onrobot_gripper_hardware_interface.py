#!/usr/bin/env python3

import rclpy
from threading import Lock
import time

# ROS2 Control imports for Humble - CORRECT IMPORTS
from hardware_interface.system_interface import SystemInterface
from hardware_interface.types import LifecycleState
from hardware_interface.hardware_info import HardwareInfo
from hardware_interface.callback_return import CallbackReturn

class OnRobotGripperHardwareInterface(SystemInterface):
    """
    ROS2 Control Hardware Interface for OnRobot Grippers
    """
    
    def __init__(self):
        super().__init__()
        self.node_ = None
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
        
        # Hardware connection
        self.gripper_driver_ = None
        self.simulation_mode_ = True
        
    def on_init(self, info: HardwareInfo):
        """
        Initialize the hardware interface
        """
        try:
            # Get parameters from hardware info
            self.ip_address_ = info.hardware_parameters.get('ip_address', '192.168.1.1')
            self.port_ = int(info.hardware_parameters.get('port', '502'))
            self.gripper_type_ = info.hardware_parameters.get('gripper_type', '2FG7')
            self.max_width_ = float(info.hardware_parameters.get('max_width', '0.085'))
            self.min_width_ = float(info.hardware_parameters.get('min_width', '0.0'))
            self.max_force_ = float(info.hardware_parameters.get('max_force', '100.0'))
            
            print(f"OnRobot Gripper Hardware Interface initialized:")
            print(f"  IP: {self.ip_address_}, Type: {self.gripper_type_}")
            print(f"  Width: {self.min_width_:.3f}m to {self.max_width_:.3f}m")
            
            return CallbackReturn.SUCCESS
            
        except Exception as e:
            print(f"❌ Error initializing hardware interface: {e}")
            return CallbackReturn.ERROR

    def on_configure(self, previous_state):
        """
        Configure the hardware interface
        """
        try:
            # For now, always use simulation mode in hardware interface
            # The real hardware communication is handled by the separate driver node
            self.simulation_mode_ = True
            print("✅ Configured OnRobot Gripper Hardware Interface (Simulation Mode)")
            return CallbackReturn.SUCCESS
        except Exception as e:
            print(f"❌ Error configuring hardware interface: {e}")
            return CallbackReturn.ERROR
    
    def on_activate(self, previous_state):
        """
        Activate the hardware interface
        """
        try:
            print("✅ Activating OnRobot Gripper Hardware Interface")
            # Initialize to open position
            self.position_ = self.max_width_ / 2
            self.command_position_ = self.max_width_ / 2
            self.last_command_position_ = self.max_width_ / 2
            
            print("✅ Hardware interface ready for simulation")
            return CallbackReturn.SUCCESS
        except Exception as e:
            print(f"❌ Error activating hardware interface: {e}")
            return CallbackReturn.ERROR
    
    def on_deactivate(self, previous_state):
        """
        Deactivate the hardware interface
        """
        try:
            print("✅ Deactivating OnRobot Gripper Hardware Interface")
            return CallbackReturn.SUCCESS
        except Exception as e:
            print(f"❌ Error deactivating hardware interface: {e}")
            return CallbackReturn.ERROR
    
    def on_cleanup(self):
        """
        Cleanup the hardware interface
        """
        try:
            print("✅ Cleaned up OnRobot Gripper Hardware Interface")
            return CallbackReturn.SUCCESS
        except Exception as e:
            print(f"❌ Error cleaning up hardware interface: {e}")
            return CallbackReturn.ERROR
    
    def on_shutdown(self):
        """
        Shutdown the hardware interface
        """
        try:
            return self.on_cleanup()
        except Exception as e:
            print(f"❌ Error shutting down hardware interface: {e}")
            return CallbackReturn.ERROR
    
    def on_error(self, previous_state):
        """
        Handle errors
        """
        print("❌ OnRobot Gripper Hardware Interface entered error state")
        return CallbackReturn.SUCCESS
    
    def export_state_interfaces(self):
        """
        Export state interfaces for ROS2 Control
        """
        interfaces = []
        
        # Add state interfaces for both finger joints
        interfaces.append({
            'name': 'left_finger_joint',
            'interface': 'position',
            'data': self.position_
        })
        interfaces.append({
            'name': 'left_finger_joint',
            'interface': 'velocity', 
            'data': self.velocity_
        })
        interfaces.append({
            'name': 'left_finger_joint',
            'interface': 'effort',
            'data': self.effort_
        })

        interfaces.append({
            'name': 'right_finger_joint',
            'interface': 'position',
            'data': self.position_  # Symmetrical movement
        })
        interfaces.append({
            'name': 'right_finger_joint', 
            'interface': 'velocity',
            'data': self.velocity_
        })
        interfaces.append({
            'name': 'right_finger_joint',
            'interface': 'effort', 
            'data': self.effort_
        })
        
        return interfaces

    def export_command_interfaces(self):
        """
        Export command interfaces for ROS2 Control
        """
        interfaces = []
                
        # Add command interface for position control
        interfaces.append({
            'name': 'left_finger_joint',
            'interface': 'position', 
            'data': self.command_position_
        })

        # For symmetrical gripper, both joints should move together
        interfaces.append({
            'name': 'right_finger_joint',
            'interface': 'position', 
            'data': self.command_position_
        })

        return interfaces
        
    def read(self):
        """
        Read data from hardware
        """
        try:
            with self.lock_:
                # In simulation mode, simulate reading from hardware
                # Gradually move toward commanded position
                if abs(self.position_ - self.command_position_) > 0.001:
                    step = (self.command_position_ - self.position_) * 0.1
                    self.position_ += step
                    self.velocity_ = abs(step) * 10  # Simulate velocity
                else:
                    self.velocity_ = 0.0
                
                # Simulate effort based on movement
                self.effort_ = self.max_force_ * 0.3  # Constant effort in simulation
            
            return CallbackReturn.SUCCESS
            
        except Exception as e:
            print(f"❌ Error reading from hardware: {e}")
            return CallbackReturn.ERROR
    
    def write(self):
        """
        Write commands to hardware
        """
        try:
            with self.lock_:
                # In simulation, just update the last command
                if abs(self.command_position_ - self.last_command_position_) > 0.001:
                    print(f"✅ Simulated gripper command: position={self.command_position_:.3f}m")
                    self.last_command_position_ = self.command_position_
            
            return CallbackReturn.SUCCESS
            
        except Exception as e:
            print(f"❌ Error writing to hardware: {e}")
            return CallbackReturn.ERROR