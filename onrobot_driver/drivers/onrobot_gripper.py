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
        self.node.declare_parameter('max_width', 0.070)    # 70mm max opening
        self.node.declare_parameter('min_width', 0.035)    # 35mm min opening
        self.node.declare_parameter('max_force', 140.0)
        self.node.declare_parameter('update_rate', 100.0)
        self.node.declare_parameter('simulation_mode', False)
        self.node.declare_parameter('joint_name', 'left_finger_joint')
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
        
        # Gripper state initialization - start at 55mm (middle position)
        self.current_position = 0.055  # 55mm initial opening
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
        
        # Publish initial joint state immediately to prevent MoveIt warnings
        self.publish_initial_joint_state()
        
        # Setup communication
        self.setup_communication()
        
        self.logger.info(f"OnRobot {self.gripper_type} gripper initialized")
        self.logger.info(f"Mode: {'SIMULATION' if self.simulation_mode else 'HARDWARE'}")
        self.logger.info(f"IP: {self.ip_address}, Port: {self.port}")
        self.logger.info(f"Width range: {self.min_width:.3f}m to {self.max_width:.3f}m")
        self.logger.info(f"Joint name: {self.joint_name}")
    
    def publish_initial_joint_state(self):
        """
        Publish initial joint state immediately to prevent MoveIt warnings.
        """
        try:
            joint_state = JointState()
            joint_state.header.stamp = self.node.get_clock().now().to_msg()
            joint_state.header.frame_id = ""  # Empty frame id
            joint_state.name = ['left_finger_joint', 'right_finger_joint']
            
            # Start at 55mm opening
            left_pos = self.meters_to_joint_position(0.055)
            right_pos = -left_pos
            
            joint_state.position = [left_pos, right_pos]
            joint_state.velocity = [0.0, 0.0]
            joint_state.effort = [0.0, 0.0]
            
            # Publish immediately and a few more times to ensure reception
            for i in range(3):
                self.joint_state_pub.publish(joint_state)
                time.sleep(0.1)
                
            self.logger.info("Initial gripper joint state published")
            
        except Exception as e:
            self.logger.error(f"Error publishing initial joint state: {e}")
    
    def setup_communication(self):
        """Setup communication with register discovery"""
        if self.simulation_mode:
            self.logger.info("Running in simulation mode - no hardware connection")
            self.is_connected = True
            return
            
        try:
            if self.client and self.client.connect():
                self.is_connected = True
                self.logger.info("✅ Connected to OnRobot Compute Box")
                
                # Discover gripper registers
                valid_registers = self.client.scan_onrobot_registers()
                
                if valid_registers:
                    self.logger.info("✅ Gripper register discovery successful")
                    # Test communication
                    if self.read_gripper_status():
                        self.logger.info("✅ Gripper communication established")
                    else:
                        self.logger.warning("⚠️ Initial status read failed, but continuing")
                else:
                    self.logger.error("❌ No gripper registers found - check power and model")
                    self.logger.error("Switching to simulation mode")
                    self.simulation_mode = True
            else:
                self.logger.error(f"❌ Failed to connect to {self.ip_address}:{self.port}")
                self.logger.error("Switching to simulation mode")
                self.simulation_mode = True
                self.is_connected = True
                    
        except Exception as e:
            self.logger.error(f"❌ Hardware connection failed: {e}")
            self.logger.error("Switching to simulation mode")
            self.simulation_mode = True
            self.is_connected = True
    
    def setup_ros_interfaces(self):
        """Setup ROS2 publishers, subscribers, and action servers"""
        # FIXED: Publish to the main joint_states topic that MoveIt expects
        self.joint_state_pub = self.node.create_publisher(JointState, 'joint_states', 10)
        
        # Additional status topics for monitoring
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
        Send command to OnRobot gripper using proper protocol.
        """
        if not self.client or not self.client.is_connected:
            self.logger.error("No connection to gripper hardware")
            return False
        
        try:
            # OnRobot 2FG7 command protocol
            # Method 1: Write to command registers (most common)
            success = self.client.write_multiple_registers(0x1000, [position, force])
            if success:
                self.logger.debug(f"Command sent: position={position}, force={force}")
                return True
            
            # Method 2: Try single register writes
            success = self.client.write_single_register(0x1000, position)
            if success:
                self.logger.debug(f"Position command sent: {position}")
                # Also send force if possible
                self.client.write_single_register(0x1001, force)
                return True
            
            # Method 3: Try alternative addresses
            for addr in [0x0000, 0x2000]:
                success = self.client.write_single_register(addr, position)
                if success:
                    self.logger.debug(f"Command sent via 0x{addr:04X}")
                    return True
            
            self.logger.error("All command methods failed - check gripper model and registers")
            return False
                
        except Exception as e:
            self.logger.error(f"Error sending gripper command: {e}")
            return False
    
    def read_gripper_status(self) -> bool:
        """
        Read current gripper status using OnRobot-specific protocol.
        """
        if self.simulation_mode or not self.client or not self.client.is_connected:
            # For simulation, just return current state
            self.is_ready = True
            return True
        
        try:
            # OnRobot 2FG7 specific register addresses
            # Try status register first
            status_data = self.client.read_holding_registers(0x0000, 1)
            if status_data:
                status = status_data[0]
                self.is_ready = (status & 0x0001) != 0  # Bit 0: Ready
                self.is_moving = (status & 0x0002) != 0  # Bit 1: Moving
                
                # Read position and force if available
                pos_data = self.client.read_holding_registers(0x0001, 1)
                if pos_data:
                    self.current_position = self.units_to_meters(pos_data[0])
                
                force_data = self.client.read_holding_registers(0x0002, 1)
                if force_data:
                    self.current_force = self.units_to_force(force_data[0])
                
                self.logger.debug(f"Status: ready={self.is_ready}, moving={self.is_moving}, "
                                f"position={self.current_position:.3f}m")
                return True
            
            # Alternative register mapping
            status_data = self.client.read_holding_registers(0x1000, 3)
            if status_data and len(status_data) >= 3:
                return self.parse_onrobot_status(status_data)
            
            self.logger.warning("No status data received from OnRobot gripper")
            return False
                
        except Exception as e:
            self.logger.error(f"Error reading gripper status: {e}")
            return False

    def parse_onrobot_status(self, data: list) -> bool:
        """
        Parse OnRobot-specific status data.
        """
        try:
            # OnRobot status register mapping
            status = data[0]
            position_raw = data[1] if len(data) > 1 else 0
            force_raw = data[2] if len(data) > 2 else 0
            
            # Parse status bits for OnRobot grippers
            self.is_ready = (status & 0x0001) != 0    # Ready flag
            self.is_moving = (status & 0x0002) != 0   # Moving flag
            object_detected = (status & 0x0004) != 0  # Object detected
            
            # Convert raw values
            self.current_position = self.units_to_meters(position_raw)
            self.current_force = self.units_to_force(force_raw)
            
            self.logger.debug(f"OnRobot Status: ready={self.is_ready}, moving={self.is_moving}, "
                             f"object_detected={object_detected}, pos={self.current_position:.3f}m")
            
            return True
            
        except Exception as e:
            self.logger.error(f"Error parsing OnRobot status: {e}")
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
    
    def publish_status(self):
        """
        Publish current gripper status to ROS2 topics.
        FIXED: Now publishes BOTH finger joints to /joint_states topic
        """
        try:
            # FIXED: Publish to the MAIN joint_states topic that MoveIt expects
            joint_state = JointState()
            joint_state.header.stamp = self.node.get_clock().now().to_msg()
            
            # Include BOTH finger joints
            joint_state.name = ['left_finger_joint', 'right_finger_joint']
            
            # Convert width to joint positions for 2FG7:
            # - Total width = left_position - right_position  
            # - For 55mm (0.055m) opening: left=0.0075, right=-0.0075
            # - For 70mm (0.070m) opening: left=0.0175, right=-0.0175  
            # - For 35mm (0.035m) opening: left=0.0, right=0.0
            left_pos = self.meters_to_joint_position(self.current_position)
            right_pos = -left_pos  # Mirror for right finger
            
            joint_state.position = [left_pos, right_pos]
            joint_state.velocity = [0.0, 0.0]  # We don't have velocity data
            joint_state.effort = [self.current_force, self.current_force]
            
            # Publish to the main joint_states topic
            self.joint_state_pub.publish(joint_state)
            
            # Also publish individual status topics for monitoring
            status_msg = Bool()
            status_msg.data = self.is_ready
            self.status_pub.publish(status_msg)
            
            position_msg = Float32()
            position_msg.data = self.current_position
            self.position_pub.publish(position_msg)
            
            connection_msg = Bool()
            connection_msg.data = self.is_connected
            self.connection_pub.publish(connection_msg)
            
            mode_msg = String()
            mode_msg.data = 'simulation' if self.simulation_mode else 'hardware'
            self.mode_pub.publish(mode_msg)
            
        except Exception as e:
            self.logger.error(f"Error publishing status: {e}")
    
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

    def meters_to_joint_position(self, meters: float) -> float:
        """
        Convert gripper width in meters to left_finger_joint position.
        Range: 0.035m (closed) to 0.070m (open)
        Joint range: 0.0 to 0.0175
        """
        if self.max_width == self.min_width:
            return 0.0
        normalized = (meters - self.min_width) / (self.max_width - self.min_width)
        return normalized * 0.0175
    
    def joint_position_to_meters(self, joint_pos: float) -> float:
        """
        Convert left_finger_joint position to gripper width in meters.
        """
        normalized = joint_pos / 0.0175
        return self.min_width + normalized * (self.max_width - self.min_width)
    
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