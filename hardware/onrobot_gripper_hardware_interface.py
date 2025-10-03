#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from threading import Lock
import time

# ROS2 Control imports for Humble - FIXED IMPORTS
try:
    # Try the standard ROS2 Humble imports
    from hardware_interface.system_interface import SystemInterface
    from hardware_interface.types import LifecycleState
    from hardware_interface.hardware_info import HardwareInfo
    from hardware_interface.callback_return import CallbackReturn
    print("✅ Using standard hardware_interface imports")
except ImportError:
    try:
        # Try alternative import paths
        from ros2_control.hardware_interface import SystemInterface, CallbackReturn
        from ros2_control.hardware_interface import LifecycleState
        from ros2_control.hardware_interface import HardwareInfo
        print("✅ Using ros2_control hardware_interface imports")
    except ImportError:
        # Fallback implementation for testing
        print("⚠️ Using fallback implementation - ROS2 Control not found")
        from enum import Enum
        
        class CallbackReturn(Enum):
            SUCCESS = 0
            ERROR = 1
            FAILURE = 2
        
        class LifecycleState(Enum):
            UNCONFIGURED = 0
            INACTIVE = 1
            ACTIVE = 2
            FINALIZED = 3
        
        class HardwareInfo:
            def __init__(self):
                self.hardware_parameters = {}
        
        class SystemInterface:
            def __init__(self):
                pass

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
        
        # Simulation mode
        self.simulation_mode_ = True
        
    def on_init(self, hardware_info):
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
            
            # Auto-detect simulation mode
            self.simulation_mode_ = self._detect_simulation_mode()
            
            print(f"OnRobot Gripper Hardware Interface initialized: "
                  f"IP: {self.ip_address_}, Type: {self.gripper_type_}, "
                  f"Simulation: {self.simulation_mode_}")
            
            return CallbackReturn.SUCCESS
            
        except Exception as e:
            print(f"Error initializing hardware interface: {e}")
            return CallbackReturn.ERROR

    def _detect_simulation_mode(self):
        """
        Auto-detect if we should run in simulation mode
        """
        # Force simulation if using localhost
        if self.ip_address_ in ["127.0.0.1", "localhost"]:
            return True
            
        # If using hardware IP but cannot connect, fallback to simulation
        try:
            import socket
            test_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            test_socket.settimeout(1.0)  # 1 second timeout
            result = test_socket.connect_ex((self.ip_address_, self.port_))
            test_socket.close()
            
            if result != 0:
                print(f"⚠️ Cannot reach {self.ip_address_}:{self.port_} - falling back to simulation mode")
                return True
            else:
                print(f"✅ Successfully connected to {self.ip_address_}:{self.port_} - using hardware mode")
                return False
        except ImportError:
            print("⚠️ Socket module not available - falling back to simulation mode")
            return True
        except Exception as e:
            print(f"⚠️ Connection test failed: {e} - falling back to simulation mode")
            return True
    
    def on_configure(self, previous_state):
        """
        Configure the hardware interface
        """
        try:
            # Initialize ROS node for the gripper driver
            if not rclpy.ok():
                rclpy.init()
            
            self.node_ = Node('onrobot_gripper_hardware_interface')
            
            # In simulation mode, we don't need actual hardware connection
            if not self.simulation_mode_:
                try:
                    from onrobot_driver.drivers.onrobot_gripper import OnRobotGripper
                    self.gripper_ = OnRobotGripper(self.node_)
                    print("✅ OnRobotGripper driver initialized for hardware mode")
                except ImportError as e:
                    print(f"❌ Could not import OnRobotGripper: {e}")
                    print("⚠️ Falling back to simulation mode")
                    self.simulation_mode_ = True
            else:
                print("✅ Running in simulation mode - no hardware connection")
            
            print("✅ Configured OnRobot Gripper Hardware Interface")
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
            
            if self.gripper_ and not self.simulation_mode_:
                print("✅ Hardware interface ready for real gripper commands")
            else:
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
            if self.node_:
                self.node_.destroy_node()
            if self.gripper_:
                self.gripper_.disconnect()
            if rclpy.ok():
                rclpy.shutdown()
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
                if self.gripper_ and not self.simulation_mode_:
                    # Read actual position from gripper
                    self.position_ = self.gripper_.current_position
                    # For simulation, you might want to get velocity and effort from the driver
                    self.velocity_ = 0.0  # Could be computed from position changes
                    self.effort_ = self.gripper_.current_force
                else:
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
                # Only send command if position has changed significantly
                if (self.gripper_ and not self.simulation_mode_ and 
                    abs(self.command_position_ - self.last_command_position_) > 0.001):
                    
                    success = self.gripper_.move_to_position(
                        self.command_position_, 
                        self.max_force_
                    )
                    
                    if success:
                        self.last_command_position_ = self.command_position_
                        print(f"✅ Sent command to gripper: position={self.command_position_:.3f}m")
                elif self.simulation_mode_:
                    # In simulation, just update the last command
                    if abs(self.command_position_ - self.last_command_position_) > 0.001:
                        print(f"✅ Simulated gripper command: position={self.command_position_:.3f}m")
                        self.last_command_position_ = self.command_position_
            
            return CallbackReturn.SUCCESS
            
        except Exception as e:
            print(f"❌ Error writing to hardware: {e}")
            return CallbackReturn.ERROR