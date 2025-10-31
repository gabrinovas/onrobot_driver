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
    Unified OnRobot gripper controller for 2FG7/2FG14 grippers.
    CORRECTED for asymmetric URDF configuration.
    Based on actual Modbus register mapping from documentation.
    """
    
    # Register addresses from documentation
    REG_TARGET_WIDTH = 0x0000    # Target width in 1/10 mm
    REG_TARGET_FORCE = 0x0001    # Target force in N
    REG_TARGET_SPEED = 0x0002    # Target speed in % (10-100%)
    REG_COMMAND = 0x0003         # Command register
    REG_STATUS = 0x0100          # Status register
    REG_EXTERNAL_WIDTH = 0x0101  # External width in 1/10 mm signed
    REG_INTERNAL_WIDTH = 0x0102  # Internal width in 1/10 mm signed
    REG_FORCE = 0x0107           # Current force in N
    
    # Command codes from documentation
    CMD_GRIP_EXTERNAL = 1
    CMD_GRIP_INTERNAL = 2
    CMD_STOP = 3
    
    # Status bit masks from documentation
    STATUS_BUSY = 0x0001
    STATUS_GRIP_DETECTED = 0x0002
    STATUS_ERROR_NOT_CALIBRATED = 0x0008
    STATUS_ERROR_LINEAR_SENSOR = 0x0010
    
    def __init__(self, node: Node):
        self.node = node
        self.logger = node.get_logger()
        
        # Declare parameters with defaults
        self.node.declare_parameter('gripper_type', '2FG7')
        self.node.declare_parameter('ip_address', '192.168.1.1')
        self.node.declare_parameter('port', 502)
        self.node.declare_parameter('max_width', 0.070)    # 70mm max opening
        self.node.declare_parameter('min_width', 0.035)    # 35mm min opening
        self.node.declare_parameter('max_force', 100.0)
        self.node.declare_parameter('update_rate', 100.0)
        self.node.declare_parameter('simulation_mode', False)
        self.node.declare_parameter('joint_name', 'left_finger_joint')
        self.node.declare_parameter('command_timeout', 10.0)
        self.node.declare_parameter('device_address', 65)  # Default for Quick Changer
        
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
        self.device_address = self.node.get_parameter('device_address').value
        
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
        self.last_command_time = self.node.get_clock().now()
        self.lock = Lock()
        
        # Hardware communication (only initialized if needed)
        self.client = None
        if not self.simulation_mode:
            try:
                self.client = ModbusTCPClient(self.ip_address, self.port, 
                                            unit_id=self.device_address, timeout=2.0)
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
        self.logger.info(f"IP: {self.ip_address}, Port: {self.port}, Device: {self.device_address}")
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
        self.stop()
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
        Move gripper to specified position using external grip command.
        Handles both simulation and hardware modes.
        """
        with self.lock:
            # Validate inputs and clamp to allowed range
            position = max(self.min_width, min(self.max_width, position))
            force = min(self.max_force, max(0.0, force))
            
            self.logger.info(f"Moving gripper to: {position:.3f}m, force: {force:.1f}N")
            
            if not self.simulation_mode and self.client and self.client.is_connected:
                # Hardware mode: send command to hardware using proper protocol
                width_units = self.meters_to_units(position)
                force_units = int(force)  # Force is in N directly
                
                success = self.send_gripper_command(width_units, force_units)
                if success:
                    self.target_position = position
                    self.is_moving = True
                    self.last_command_time = self.node.get_clock().now()
                    return True
                else:
                    self.logger.error("Failed to send command to gripper hardware")
                    return False
            else:
                # Simulation mode: simulate movement
                self.simulate_movement(position, force)
                return True
    
    def send_gripper_command(self, width_units: int, force_units: int) -> bool:
        """
        Send command to real hardware using proper OnRobot protocol.
        Returns True if command was sent successfully.
        """
        if not self.client or not self.client.is_connected:
            self.logger.error("No connection to gripper hardware")
            return False
        
        try:
            # According to documentation, we need to write to multiple registers:
            # 1. Target width (0x0000) in 1/10 mm
            # 2. Target force (0x0001) in N
            # 3. Command (0x0003) with grip external command
            
            # Write target width
            success1 = self.client.write_single_register(self.REG_TARGET_WIDTH, width_units)
            
            # Write target force
            success2 = self.client.write_single_register(self.REG_TARGET_FORCE, force_units)
            
            # Write command to start movement
            success3 = self.client.write_single_register(self.REG_COMMAND, self.CMD_GRIP_EXTERNAL)
            
            if success1 and success2 and success3:
                self.logger.debug("Gripper command sent successfully")
                return True
            else:
                self.logger.error(f"Command failed: width={success1}, force={success2}, cmd={success3}")
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
            # Read status register (0x0100)
            status_data = self.client.read_holding_registers(self.REG_STATUS, 1)
            if not status_data:
                return False
                
            status = status_data[0]
            
            # Parse status bits
            self.is_moving = (status & self.STATUS_BUSY) != 0
            grip_detected = (status & self.STATUS_GRIP_DETECTED) != 0
            error_calibration = (status & self.STATUS_ERROR_NOT_CALIBRATED) != 0
            error_sensor = (status & self.STATUS_ERROR_LINEAR_SENSOR) != 0
            
            # Update readiness
            self.is_ready = not (error_calibration or error_sensor)
            
            # Read current width (external width register 0x0101)
            width_data = self.client.read_holding_registers(self.REG_EXTERNAL_WIDTH, 1)
            if width_data:
                # Convert from 1/10 mm to meters
                width_units = width_data[0]
                # Handle signed value for external width
                if width_units & 0x8000:  # Check if negative (two's complement)
                    width_units = -((width_units ^ 0xFFFF) + 1)
                self.current_position = width_units / 10000.0  # Convert 1/10 mm to meters
            
            # Read current force (register 0x0107)
            force_data = self.client.read_holding_registers(self.REG_FORCE, 1)
            if force_data:
                self.current_force = float(force_data[0])  # Force is in N
            
            # Check if we've reached target position (with tolerance)
            if not self.is_moving and hasattr(self, 'target_position'):
                tolerance = 0.002  # 2mm tolerance
                if abs(self.current_position - self.target_position) < tolerance:
                    self.is_moving = False
            
            # Log errors if any
            if error_calibration:
                self.logger.error("Gripper not calibrated!")
            if error_sensor:
                self.logger.error("Linear sensor error!")
                
            return True
                
        except Exception as e:
            self.logger.error(f"Error reading gripper status: {e}")
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
        CORRECTED for asymmetric URDF configuration.
        """
        try:
            # Your URDF has asymmetric finger positions:
            # Left finger: moves from 0.032239 to 0.014739 (0.0 to 0.0175 in joint space)
            # Right finger: moves from -0.054361 to -0.036861 (-0.0175 to 0.0 in joint space)
            
            # Convert driver position (0.035-0.070m total opening) to joint positions
            total_opening = self.current_position  # This is 0.035 to 0.070 from driver
            
            # Calculate finger displacement from closed position
            # Closed: 0.035m opening, each finger at 0.0 in joint space
            # Open: 0.070m opening, each finger at 0.0175 in joint space
            finger_displacement = (total_opening - 0.035) / 2.0
            
            # Left finger: starts at 0.032239, moves negative X direction
            left_finger_position = 0.032239 - finger_displacement
            
            # Right finger: starts at -0.054361, moves positive X direction  
            right_finger_position = -0.054361 + finger_displacement
            
            joint_state = JointState()
            joint_state.header.stamp = self.node.get_clock().now().to_msg()
            joint_state.name = ['left_finger_joint', 'right_finger_joint']
            
            # Joint space positions (what the controller expects)
            # Left: 0.0 (closed) to 0.0175 (open)
            # Right: -0.0175 (closed) to 0.0 (open)
            joint_space_left = finger_displacement
            joint_space_right = -finger_displacement
            
            joint_state.position = [joint_space_left, joint_space_right]
            joint_state.velocity = [0.0, 0.0]
            joint_state.effort = [self.current_force, self.current_force]
            self.joint_state_pub.publish(joint_state)
            
            # Publish additional debug info
            self.logger.debug(f"Gripper: total={total_opening:.4f}m, "
                             f"left_joint={joint_space_left:.4f}m, "
                             f"right_joint={joint_space_right:.4f}m, "
                             f"left_world={left_finger_position:.4f}m, "
                             f"right_world={right_finger_position:.4f}m")
            
            # Publish status topics
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
        Convert meters to gripper units (1/10 mm).
        """
        mm = meters * 1000.0  # Convert to mm
        tenth_mm = int(mm * 10.0)  # Convert to 1/10 mm
        return max(0, min(0xFFFF, tenth_mm))  # Clamp to valid range
    
    def units_to_meters(self, units: int) -> float:
        """
        Convert gripper units (1/10 mm) to meters.
        """
        mm = units / 10.0  # Convert to mm
        return mm / 1000.0  # Convert to meters
    
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
            
            # Check for command timeout
            if self.is_moving and not self.simulation_mode:
                time_since_command = (self.node.get_clock().now() - self.last_command_time).nanoseconds / 1e9
                if time_since_command > self.command_timeout:
                    self.logger.warning("Gripper command timeout - stopping movement")
                    self.stop()
            
        except Exception as e:
            self.logger.error(f"Error in update callback: {e}")
    
    def stop(self):
        """
        Stop any ongoing movement.
        """
        with self.lock:
            self.is_moving = False
            
            # Send stop command to hardware if connected
            if not self.simulation_mode and self.client and self.client.is_connected:
                try:
                    self.client.write_single_register(self.REG_COMMAND, self.CMD_STOP)
                    self.logger.info("Stop command sent to gripper")
                except Exception as e:
                    self.logger.error(f"Failed to send stop command: {e}")
            
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
    
    def get_status_dict(self) -> dict:
        """
        Get current gripper status as dictionary.
        Useful for monitoring and diagnostics.
        """
        # Calculate joint positions for status display
        finger_displacement = (self.current_position - 0.035) / 2.0
        joint_space_left = finger_displacement
        joint_space_right = -finger_displacement
        
        return {
            'position': self.current_position,
            'target_position': self.target_position,
            'force': self.current_force,
            'left_joint_position': joint_space_left,
            'right_joint_position': joint_space_right,
            'is_ready': self.is_ready,
            'is_moving': self.is_moving,
            'is_connected': self.is_connected,
            'simulation_mode': self.simulation_mode,
            'gripper_type': self.gripper_type,
            'min_width': self.min_width,
            'max_width': self.max_width,
            'max_force': self.max_force
        }