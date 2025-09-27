#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from threading import Lock
import time

from control_msgs.action import GripperCommand

# Correct imports for ROS2 Humble
from hardware_interface import SystemInterface
from hardware_interface import CallbackReturn

class OnRobotGripperHardwareInterface(SystemInterface):
    """
    ROS2 Control Hardware Interface for OnRobot Grippers
    """
    
    def __init__(self):
        super().__init__()
        self.node_ = None
        self.gripper_action_client_ = None
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
        
    def on_init(self, info):
        """
        Initialize the hardware interface
        """
        try:
            # Initialize ROS node if not already initialized
            if not rclpy.ok():
                rclpy.init()
            
            self.node_ = Node('onrobot_gripper_hardware_interface')
            
            # Get parameters from hardware info
            if hasattr(info, 'hardware_parameters'):
                self.ip_address_ = info.hardware_parameters.get('ip_address', '192.168.1.1')
                self.port_ = int(info.hardware_parameters.get('port', '502'))
                self.gripper_type_ = info.hardware_parameters.get('gripper_type', '2FG7')
                self.max_width_ = float(info.hardware_parameters.get('max_width', '0.085'))
                self.min_width_ = float(info.hardware_parameters.get('min_width', '0.0'))
                self.max_force_ = float(info.hardware_parameters.get('max_force', '100.0'))
            
            # Initialize action client for OnRobot driver
            self.gripper_action_client_ = ActionClient(
                self.node_, GripperCommand, '/gripper_action'
            )
            
            self.node_.get_logger().info(
                f"OnRobot Gripper Hardware Interface initialized: "
                f"IP: {self.ip_address_}, Type: {self.gripper_type_}"
            )
            
            return CallbackReturn.SUCCESS
            
        except Exception as e:
            if self.node_:
                self.node_.get_logger().error(f"Error initializing hardware interface: {e}")
            return CallbackReturn.ERROR
    
    def on_configure(self, previous_state):
        """
        Configure the hardware interface
        """
        try:
            self.node_.get_logger().info("Configuring OnRobot Gripper Hardware Interface")
            return CallbackReturn.SUCCESS
        except Exception as e:
            self.node_.get_logger().error(f"Error configuring hardware interface: {e}")
            return CallbackReturn.ERROR
    
    def on_cleanup(self):
        """
        Cleanup the hardware interface
        """
        try:
            if self.node_:
                self.node_.get_logger().info("Cleaning up OnRobot Gripper Hardware Interface")
                self.node_.destroy_node()
            rclpy.shutdown()
            return CallbackReturn.SUCCESS
        except Exception as e:
            print(f"Error cleaning up hardware interface: {e}")
            return CallbackReturn.ERROR
    
    def on_activate(self, previous_state):
        """
        Activate the hardware interface
        """
        try:
            self.node_.get_logger().info("Activating OnRobot Gripper Hardware Interface")
            
            # Wait for action server
            if not self.gripper_action_client_.wait_for_server(timeout_sec=5.0):
                self.node_.get_logger().warn("Gripper action server not available, running in simulation mode")
            
            return CallbackReturn.SUCCESS
        except Exception as e:
            self.node_.get_logger().error(f"Error activating hardware interface: {e}")
            return CallbackReturn.ERROR
    
    def on_deactivate(self, previous_state):
        """
        Deactivate the hardware interface
        """
        try:
            self.node_.get_logger().info("Deactivating OnRobot Gripper Hardware Interface")
            return CallbackReturn.SUCCESS
        except Exception as e:
            self.node_.get_logger().error(f"Error deactivating hardware interface: {e}")
            return CallbackReturn.ERROR
    
    def on_error(self, previous_state):
        """
        Handle errors
        """
        if self.node_:
            self.node_.get_logger().error("OnRobot Gripper Hardware Interface entered error state")
        return CallbackReturn.SUCCESS
    
    def on_shutdown(self, previous_state):
        """
        Shutdown the hardware interface
        """
        try:
            if self.node_:
                self.node_.get_logger().info("Shutting down OnRobot Gripper Hardware Interface")
            return CallbackReturn.SUCCESS
        except Exception as e:
            print(f"Error shutting down hardware interface: {e}")
            return CallbackReturn.ERROR
    
    def export_state_interfaces(self):
        """
        Export state interfaces for ROS2 Control
        """
        interfaces = []
        
        # Add state interfaces for left finger joint
        interfaces.append(('left_finger_joint', 'position', self.position_))
        interfaces.append(('left_finger_joint', 'velocity', self.velocity_))
        interfaces.append(('left_finger_joint', 'effort', self.effort_))
        
        # Add state interfaces for right finger joint (mirrored)
        interfaces.append(('right_finger_joint', 'position', self.position_))
        interfaces.append(('right_finger_joint', 'velocity', self.velocity_))
        interfaces.append(('right_finger_joint', 'effort', self.effort_))
        
        return interfaces
    
    def export_command_interfaces(self):
        """
        Export command interfaces for ROS2 Control
        """
        interfaces = []
        
        # Add command interface for left finger joint
        interfaces.append(('left_finger_joint', 'position', self.command_position_))
        
        return interfaces
    
    def read(self):
        """
        Read data from hardware
        """
        try:
            # For now, we'll simulate reading from hardware
            # In a real implementation, you would:
            # 1. Read current position from OnRobot driver
            # 2. Update self.position_, self.velocity_, self.effort_
            
            # Simulate reading - in real implementation, get this from hardware
            with self.lock_:
                # For simulation, just reflect the command position
                if abs(self.command_position_ - self.position_) > 0.001:
                    self.position_ = self.command_position_
                    self.velocity_ = 0.1  # Simulated velocity
                    self.effort_ = 10.0   # Simulated effort
            
            return CallbackReturn.SUCCESS
            
        except Exception as e:
            if self.node_:
                self.node_.get_logger().error(f"Error reading from hardware: {e}")
            return CallbackReturn.ERROR
    
    def write(self):
        """
        Write commands to hardware
        """
        try:
            with self.lock_:
                # Only send command if position has changed significantly
                if abs(self.command_position_ - self.last_command_position_) > 0.001:
                    self.send_gripper_command(self.command_position_)
                    self.last_command_position_ = self.command_position_
            
            return CallbackReturn.SUCCESS
            
        except Exception as e:
            if self.node_:
                self.node_.get_logger().error(f"Error writing to hardware: {e}")
            return CallbackReturn.ERROR
    
    def send_gripper_command(self, position):
        """
        Send command to OnRobot gripper via action server
        """
        try:
            if self.gripper_action_client_ and self.gripper_action_client_.wait_for_server(timeout_sec=1.0):
                goal_msg = GripperCommand.Goal()
                goal_msg.command.position = position
                goal_msg.command.max_effort = self.max_force_
                
                # Send goal asynchronously
                future = self.gripper_action_client_.send_goal_async(goal_msg)
                
                # For now, we don't wait for result to avoid blocking
                # In production, you might want to handle this differently
                
                if self.node_:
                    self.node_.get_logger().debug(f"Sent gripper command: position={position}")
            else:
                if self.node_:
                    self.node_.get_logger().warn("Gripper action server not available, command not sent")
                
        except Exception as e:
            if self.node_:
                self.node_.get_logger().error(f"Error sending gripper command: {e}")