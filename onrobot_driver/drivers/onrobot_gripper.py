import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.action.server import ServerGoalHandle
from threading import Lock
import time
import threading

from std_msgs.msg import Bool, Float32, String
from sensor_msgs.msg import JointState
from control_msgs.action import GripperCommand
from lifecycle_msgs.srv import GetState, ChangeState
from lifecycle_msgs.msg import State, Transition

from .modbus_client import ModbusTCPClient

class OnRobotGripper:
    """
    Unified OnRobot gripper controller for both simulation and hardware.
    Properly integrated with ROS2 lifecycle and control patterns.
    """
    
    def __init__(self, node: Node):
        self.node = node
        self.logger = node.get_logger()
        
        # Declare parameters with defaults
        self.node.declare_parameter('gripper_type', '2FG7')
        self.node.declare_parameter('ip_address', '192.168.1.1')
        self.node.declare_parameter('port', 502)
        self.node.declare_parameter('max_width', 0.075)    # 75mm max opening
        self.node.declare_parameter('min_width', 0.035)    # 35mm min opening
        self.node.declare_parameter('max_force', 100.0)
        self.node.declare_parameter('update_rate', 100.0)
        self.node.declare_parameter('simulation_mode', False)
        self.node.declare_parameter('joint_name', 'gripper_joint')
        self.node.declare_parameter('command_timeout', 10.0)
        
        # Get parameters from ROS2 node
        self.gripper_type = self.node.get_parameter('gripper_type').value
        self.ip_address = self.node.get_parameter('ip_address').value
        self.port = self.node.get_parameter('port').value
        self.max_width = self.node.get_parameter('max_width').value
        self.min_width = self.node.get_parameter('min_width').value
        self.max_force = self.node.get_parameter('max_force').value
        self.update_rate = self.node.get_parameter('update_rate').value
        self.simulation_mode = self.node.get_parameter('simulation_mode').value
        self.joint_name = self.node.get_parameter('joint_name').value
        self.command_timeout = self.node.get_parameter('command_timeout').value
        
        # Auto-detect simulation mode if not explicitly set
        if not self.simulation_mode and self.ip_address in ["127.0.0.1", "localhost"]:
            self.simulation_mode = True
            self.logger.info("Auto-detected simulation mode from IP address")
        
        # Gripper state initialization
        self.current_position = (self.max_width + self.min_width) / 2  # Start at middle
        self.target_position = self.current_position
        self.current_force = 0.0
        self.is_ready = True
        self.is_moving = False
        self.is_connected = False
        self.last_update_time = self.node.get_clock().now()
        self.lock = Lock()
        
        # Hardware communication (only initialized if needed)
        self.client = None
        if not self.simulation_mode:
            try:
                self.client = ModbusTCPClient(self.ip_address, self.port, timeout=2.0)
            except Exception as e:
                self.logger.error(f"Failed to initialize Modbus client: {e}")
                self.simulation_mode = True
                self.logger.info("Falling back to simulation mode")
        
        # ROS2 interfaces
        self.setup_ros_interfaces()
        
        # Setup communication
        self.setup_communication()
        
        self.logger.info(f"OnRobot {self.gripper_type} gripper initialized")
        self.logger.info(f"Mode: {'SIMULATION' if self.simulation_mode else 'HARDWARE'}")
        self.logger.info(f"IP: {self.ip_address}, Port: {self.port}")
        self.logger.info(f"Width range: {self.min_width:.3f}m to {self.max_width:.3f}m")
        self.logger.info(f"Joint name: {self.joint_name}")
    
    def setup_communication(self):
        """Setup communication - only connect to hardware if not in simulation"""
        if self.simulation_mode:
            self.logger.info("Running in simulation mode - no hardware connection")
            self.is_connected = True
            return
            
        try:
            if self.client and self.client.connect():
                self.is_connected = True
                self.logger.info("✅ Successfully connected to OnRobot Compute Box")
                
                # Test communication by reading status
                if self.read_gripper_status():
                    self.logger.info("✅ Gripper communication test successful")
                else:
                    self.logger.warning("⚠️ Gripper status read failed, but continuing")
            else:
                self.logger.error(f"Failed to connect to {self.ip_address}:{self.port}")
                self.logger.error("Switching to simulation mode")
                self.simulation_mode = True
                self.is_connected = True
                    
        except Exception as e:
            self.logger.error(f"Hardware connection failed: {e}")
            self.logger.error("Switching to simulation mode")
            self.simulation_mode = True
            self.is_connected = True
    
    def setup_ros_interfaces(self):
        """Setup ROS2 publishers, subscribers, and action servers"""
        # Publishers for gripper status
        self.joint_state_pub = self.node.create_publisher(JointState, 'joint_states', 10)
        self.status_pub = self.node.create_publisher(Bool, 'gripper_status', 10)
        self.position_pub = self.node.create_publisher(Float32, 'gripper_position', 10)
        self.connection_pub = self.node.create_publisher(Bool, 'gripper_connected', 10)
        self.mode_pub = self.node.create_publisher(String, 'gripper_mode', 10)
        
        # Action server for gripper commands
        self.action_server = ActionServer(
            self.node,
            GripperCommand,
            'gripper_action',
            self.execute_action_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        
        # Service for simple commands
        # self.command_service = self.node.create_service(
        #     GripperCommand, 'gripper_command', 
        #     self.command_service_callback
        # )
        
        # Timer for periodic updates
        self.update_timer = self.node.create_timer(1.0/self.update_rate, self.update_callback)
        
        # Timer for status publishing (slower rate)
        self.status_timer = self.node.create_timer(0.5, self.publish_status)
        
        self.logger.info("ROS2 interfaces initialized")

    def goal_callback(self, goal_request):
        """Called when a new action goal is received"""
        self.logger.info(f"Received new gripper goal: position={goal_request.command.position:.3f}m")
        return rclpy.action.GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Called when an action goal is cancelled"""
        self.logger.info("Gripper action cancelled")
        # Stop any ongoing movement
        with self.lock:
            self.is_moving = False
        return rclpy.action.CancelResponse.ACCEPT

    def execute_action_callback(self, goal_handle: ServerGoalHandle):
        """
        Execute gripper action command - works for both simulation and hardware.
        """
        goal = goal_handle.request.command
        self.logger.info(f"Executing gripper command: position={goal.position:.3f}m, max_effort={goal.max_effort:.1f}N")
        
        # Validate command
        if goal.position < self.min_width or goal.position > self.max_width:
            error_msg = f"Invalid position {goal.position:.3f}m. Must be between {self.min_width:.3f}m and {self.max_width:.3f}m"
            self.logger.error(error_msg)
            goal_handle.abort()
            result = self._create_result(False, True)
            result.stalled = True
            return result
        
        # Check if gripper is ready
        if not self.is_ready:
            self.logger.error("Gripper is not ready")
            goal_handle.abort()
            return self._create_result(False, True)
        
        # Execute the movement
        success = self.move_to_position(goal.position, goal.max_effort)
        
        if not success:
            self.logger.error("Failed to start gripper movement")
            goal_handle.abort()
            return self._create_result(False, True)
        
        # Wait for movement completion with timeout
        start_time = self.node.get_clock().now()
        timeout_duration = rclpy.duration.Duration(seconds=self.command_timeout)
        
        while self.is_moving and (self.node.get_clock().now() - start_time < timeout_duration):
            if not goal_handle.is_active:
                self.logger.info("Goal was cancelled")
                return GripperCommand.Result()
            
            # Check for preemption
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.logger.info("Goal cancelled by user")
                return self._create_result(False, False)
            
            # Provide feedback
            feedback = self._create_feedback()
            goal_handle.publish_feedback(feedback)
            
            # Small delay to prevent excessive CPU usage
            time.sleep(0.05)
        
        # Check result
        movement_success = not self.is_moving
        stalled = not movement_success
        
        if stalled:
            self.logger.warning(f"Gripper movement timed out after {self.command_timeout} seconds")
        
        result = self._create_result(movement_success, stalled)
        
        if movement_success:
            goal_handle.succeed()
            self.logger.info("Gripper action completed successfully")
        else:
            goal_handle.abort()
            self.logger.error("Gripper action failed")
        
        return result
    
    def _create_feedback(self):
        """Create feedback message for action server"""
        feedback = GripperCommand.Feedback()
        feedback.position = self.current_position
        feedback.effort = self.current_force
        feedback.reached_goal = False
        feedback.stalled = False
        return feedback
    
    def _create_result(self, reached_goal, stalled):
        """Create result message for action server"""
        result = GripperCommand.Result()
        result.position = self.current_position
        result.effort = self.current_force
        result.reached_goal = reached_goal
        result.stalled = stalled
        return result
    
    def move_to_position(self, position: float, force: float = 50.0) -> bool:
        """
        Move gripper to specified position.
        Handles both simulation and hardware modes.
        """
        with self.lock:
            # Validate inputs and clamp to allowed range
            position = max(self.min_width, min(self.max_width, position))
            force = min(self.max_force, max(0.0, force))
            
            self.logger.info(f"Moving gripper to: {position:.3f}m, force: {force:.1f}N")
            
            if not self.simulation_mode and self.client and self.client.is_connected:
                # Hardware mode: send command to hardware
                pos_units = self.meters_to_units(position)
                force_units = self.force_to_units(force)
                success = self.send_gripper_command(pos_units, force_units)
                if success:
                    self.target_position = position
                    self.is_moving = True
                    return True
                else:
                    self.logger.error("Failed to send command to gripper hardware")
                    return False
            else:
                # Simulation mode: simulate movement
                self.simulate_movement(position, force)
                return True
    
    def send_gripper_command(self, position: int, force: int) -> bool:
        """
        Send command to real hardware.
        Returns True if command was sent successfully.
        """
        if not self.client or not self.client.is_connected:
            self.logger.error("No connection to gripper hardware")
            return False
        
        try:
            # Try different command approaches for OnRobot compatibility
            
            # Approach 1: Write to command register (common for OnRobot)
            success = self.client.write_single_register(0x1000, position)
            if success:
                self.logger.debug("Command sent successfully via single register")
                return True
            
            # Approach 2: Write position and force to consecutive registers
            success = self.client.write_multiple_registers(0x1000, [position, force])
            if success:
                self.logger.debug("Command sent successfully via multiple registers")
                return True
                
            # Approach 3: Try different register addresses
            for addr in [0x0000, 0x2000, 0x3000]:
                success = self.client.write_single_register(addr, position)
                if success:
                    self.logger.debug(f"Command sent successfully via register 0x{addr:04X}")
                    return True
            
            self.logger.error("All command methods failed")
            return False
                
        except Exception as e:
            self.logger.error(f"Error sending gripper command: {e}")
            return False
    
    def read_gripper_status(self) -> bool:
        """
        Read current gripper status from hardware or simulation.
        """
        if self.simulation_mode or not self.client or not self.client.is_connected:
            # For simulation, just return current state
            self.is_ready = True
            return True
        
        try:
            # Try different register addresses for status reading
            status_addresses = [0x0000, 0x1000, 0x2000]
            
            for addr in status_addresses:
                status_data = self.client.read_holding_registers(addr, 3)  # Read status, position, force
                
                if status_data and len(status_data) >= 3:
                    return self.parse_status_data(status_data)
            
            self.logger.warning("No status data received from any register")
            return False
                
        except Exception as e:
            self.logger.error(f"Error reading gripper status: {e}")
            return False
    
    def parse_status_data(self, data: list) -> bool:
        """
        Parse status data from gripper registers.
        Updates internal state variables.
        """
        if len(data) >= 3:
            try:
                status = data[0]
                position_raw = data[1]
                force_raw = data[2]
                
                # Parse status bits (adjust based on actual protocol)
                self.is_ready = (status & 0x01) != 0  # Ready flag
                self.is_moving = (status & 0x02) != 0  # Moving flag
                
                # Convert raw values to physical units
                self.current_position = self.units_to_meters(position_raw)
                self.current_force = self.units_to_force(force_raw)
                
                # Check if we've reached target position (with tolerance)
                if not self.is_moving and hasattr(self, 'target_position'):
                    tolerance = 0.002  # 2mm tolerance
                    if abs(self.current_position - self.target_position) < tolerance:
                        self.is_moving = False
                
                self.logger.debug(f"Status: ready={self.is_ready}, moving={self.is_moving}, "
                                f"position={self.current_position:.3f}m, force={self.current_force:.1f}N")
                
                return True
                
            except Exception as e:
                self.logger.error(f"Error parsing status data: {e}")
                return False
        else:
            self.logger.warning(f"Invalid status data length: {len(data)}")
            return False
    
    def simulate_movement(self, target_position: float, force: float = 50.0):
        """
        Simulate gripper movement for simulation mode.
        Runs in a background thread.
        """
        def movement_thread():
            with self.lock:
                self.is_moving = True
                self.target_position = target_position
            
            start_position = self.current_position
            distance = abs(target_position - start_position)
            max_speed = 0.05  # m/s
            movement_time = distance / max_speed
            
            steps = max(10, int(movement_time * self.update_rate))
            step_size = (target_position - start_position) / steps
            step_duration = movement_time / steps
            
            for i in range(steps):
                with self.lock:
                    if not self.is_moving:  # Allow cancellation
                        break
                    
                    # Update position
                    self.current_position = start_position + (i + 1) * step_size
                    
                    # Simulate force (decreases as we approach target)
                    progress = (i + 1) / steps
                    self.current_force = force * (1.0 - progress * 0.8)  # Force reduces to 20% at end
                
                time.sleep(step_duration)
            
            # Ensure we end at exact target
            with self.lock:
                self.current_position = target_position
                self.current_force = 0.0
                self.is_moving = False
            
            self.logger.info(f"Simulated movement completed: {target_position:.3f}m")
        
        thread = threading.Thread(target=movement_thread)
        thread.daemon = True
        thread.start()
    
    def meters_to_units(self, meters: float) -> int:
        """
        Convert meters to gripper units (0-255).
        """
        if self.max_width == self.min_width:
            return 0
        normalized = (meters - self.min_width) / (self.max_width - self.min_width)
        return int(normalized * 255)
    
    def units_to_meters(self, units: int) -> float:
        """
        Convert gripper units (0-255) to meters.
        """
        normalized = units / 255.0
        return self.min_width + normalized * (self.max_width - self.min_width)
    
    def force_to_units(self, force: float) -> int:
        """
        Convert force (N) to gripper units (0-255).
        """
        normalized = force / self.max_force
        return int(normalized * 255)
    
    def units_to_force(self, units: int) -> float:
        """
        Convert gripper units (0-255) to force (N).
        """
        normalized = units / 255.0
        return normalized * self.max_force
    
    def update_callback(self):
        """
        Timer callback for periodic status updates and state management.
        """
        try:
            # Update gripper status
            if not self.read_gripper_status():
                # If status read failed in hardware mode, handle gracefully
                if not self.simulation_mode:
                    self.logger.warning("Failed to read gripper status")
            
            # Update last update time
            self.last_update_time = self.node.get_clock().now()
            
        except Exception as e:
            self.logger.error(f"Error in update callback: {e}")
    
    def publish_status(self):
        """
        Publish current gripper status to ROS2 topics.
        """
        try:
            # Publish joint state
            joint_state = JointState()
            joint_state.header.stamp = self.node.get_clock().now().to_msg()
            joint_state.name = [self.joint_name]
            joint_state.position = [self.current_position]
            joint_state.velocity = [0.0]  # We don't have velocity data
            joint_state.effort = [self.current_force]
            self.joint_state_pub.publish(joint_state)
            
            # Publish status
            status_msg = Bool()
            status_msg.data = self.is_ready
            self.status_pub.publish(status_msg)
            
            # Publish position
            position_msg = Float32()
            position_msg.data = self.current_position
            self.position_pub.publish(position_msg)
            
            # Publish connection status
            connection_msg = Bool()
            connection_msg.data = self.is_connected
            self.connection_pub.publish(connection_msg)
            
            # Publish mode
            mode_msg = String()
            mode_msg.data = 'simulation' if self.simulation_mode else 'hardware'
            self.mode_pub.publish(mode_msg)
            
        except Exception as e:
            self.logger.error(f"Error publishing status: {e}")
    
    def stop(self):
        """
        Stop any ongoing movement.
        """
        with self.lock:
            self.is_moving = False
            self.logger.info("Gripper movement stopped")
    
    def disconnect(self):
        """
        Cleanup and disconnect from hardware.
        """
        self.logger.info("Disconnecting gripper...")
        
        # Stop any ongoing movement
        self.stop()
        
        # Disconnect from hardware
        if self.client and self.client.is_connected:
            self.client.disconnect()
            self.logger.info("Disconnected from OnRobot Compute Box")
        
        self.is_connected = False
    
    def emergency_stop(self):
        """
        Emergency stop function.
        """
        self.logger.warning("EMERGENCY STOP triggered")
        self.stop()
        
        # If connected to hardware, send emergency stop command
        if not self.simulation_mode and self.client and self.client.is_connected:
            try:
                # Try to send emergency stop command
                self.client.write_single_register(0x0000, 0x0000)  # Common emergency stop register
                self.logger.info("Emergency stop command sent to hardware")
            except Exception as e:
                self.logger.error(f"Failed to send emergency stop command: {e}")
    
    def get_status_dict(self) -> dict:
        """
        Get current gripper status as dictionary.
        Useful for monitoring and diagnostics.
        """
        return {
            'position': self.current_position,
            'target_position': self.target_position,
            'force': self.current_force,
            'is_ready': self.is_ready,
            'is_moving': self.is_moving,
            'is_connected': self.is_connected,
            'simulation_mode': self.simulation_mode,
            'gripper_type': self.gripper_type,
            'min_width': self.min_width,
            'max_width': self.max_width,
            'max_force': self.max_force
        }