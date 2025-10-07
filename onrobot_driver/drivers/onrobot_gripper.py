import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from threading import Lock
import time
import threading

from std_msgs.msg import Bool, Float32
from sensor_msgs.msg import JointState
from control_msgs.action import GripperCommand

from .modbus_client import ModbusTCPClient

class OnRobotGripper:
    """
    Unified OnRobot gripper controller for both simulation and hardware
    """
    
    def __init__(self, node: Node):
        self.node = node
        self.logger = node.get_logger()
        
        # Declare parameters with defaults
        self.node.declare_parameter('gripper_type', '2FG7')
        self.node.declare_parameter('ip_address', '192.168.1.1')
        self.node.declare_parameter('port', 502)
        # CORRECTED gripper parameters for 75mm-35mm range
        self.node.declare_parameter('max_width', 0.075)    # 75mm max opening
        self.node.declare_parameter('min_width', 0.035)    # 35mm min opening

        self.node.declare_parameter('max_force', 100.0)
        self.node.declare_parameter('update_rate', 100.0)
        self.node.declare_parameter('simulation_mode', False)
        
        # Get parameters
        self.gripper_type = self.node.get_parameter('gripper_type').value
        self.ip_address = self.node.get_parameter('ip_address').value
        self.port = self.node.get_parameter('port').value
        self.max_width = self.node.get_parameter('max_width').value  # 0.075
        self.min_width = self.node.get_parameter('min_width').value  # 0.035
        self.max_force = self.node.get_parameter('max_force').value
        self.update_rate = self.node.get_parameter('update_rate').value
        self.simulation_mode = self.node.get_parameter('simulation_mode').value
        
        # Auto-detect simulation mode if not explicitly set
        if not self.simulation_mode and self.ip_address in ["127.0.0.1", "localhost"]:
            self.simulation_mode = True
            self.logger.info("Auto-detected simulation mode from IP address")
        
        # Gripper state

        # Start at mid position
        middle_position = (self.max_width + self.min_width) / 2  # 0.055
        self.current_position = middle_position
        self.target_position = middle_position
        self.current_force = 0.0
        self.is_ready = True
        self.is_moving = False
        self.lock = Lock()
        
        # Hardware communication (only initialized if needed)
        self.client = None
        if not self.simulation_mode:
            try:
                self.client = ModbusTCPClient(self.ip_address, self.port)
            except Exception as e:
                self.logger.error(f"Failed to initialize Modbus client: {e}")
                self.simulation_mode = True
        
        # ROS2 interfaces
        self.setup_ros_interfaces()
        
        # Setup communication
        self.setup_communication()
        
        self.logger.info(f"OnRobot {self.gripper_type} gripper initialized")
        self.logger.info(f"Mode: {'SIMULATION' if self.simulation_mode else 'HARDWARE'}")
        self.logger.info(f"IP: {self.ip_address}, Port: {self.port}")
        self.logger.info(f"Width range: {self.min_width:.3f}m to {self.max_width:.3f}m")
    
    def setup_communication(self):
        """Setup communication - only connect to hardware if not in simulation"""
        if self.simulation_mode:
            self.logger.info("Running in simulation mode - no hardware connection")
            return
            
        try:
            if not self.client.connect():
                self.logger.error(f"Failed to connect to {self.ip_address}:{self.port}")
                self.logger.error("Switching to simulation mode")
                self.simulation_mode = True
            else:
                self.logger.info("✅ Successfully connected to OnRobot Compute Box")
                
                # Test communication by reading status
                if self.read_gripper_status():
                    self.logger.info("✅ Gripper communication test successful")
                else:
                    self.logger.warning("⚠️ Gripper status read failed, but continuing")
                    
        except Exception as e:
            self.logger.error(f"Hardware connection failed: {e}")
            self.logger.error("Switching to simulation mode")
            self.simulation_mode = True
    
    def setup_ros_interfaces(self):
        """Setup ROS2 publishers, subscribers, and action servers"""
        # Publishers
        # self.joint_state_pub = self.node.create_publisher(JointState, 'joint_states', 10)
        self.status_pub = self.node.create_publisher(Bool, 'gripper_status', 10)
        self.position_pub = self.node.create_publisher(Float32, 'gripper_position', 10)
        
        # Action server for gripper commands
        self.action_server = ActionServer(
            self.node,
            GripperCommand,
            'gripper_action',
            self.execute_action_callback
        )
        
        # Timer for periodic updates
        self.timer = self.node.create_timer(1.0/self.update_rate, self.timer_callback)
        
        self.logger.info("ROS2 interfaces initialized")

    def execute_action_callback(self, goal_handle):
        """Execute gripper action command - works for both simulation and hardware"""
        goal = goal_handle.request.command
        self.logger.info(f"Executing gripper command: position={goal.position:.3f}m, force={goal.max_effort:.1f}%")
        
        # Validate command
        if goal.position < self.min_width or goal.position > self.max_width:
            self.logger.error(f"Invalid position {goal.position:.3f}m. Must be between {self.min_width:.3f}m and {self.max_width:.3f}m")
            goal_handle.abort()
            return self._create_result(False, True)
        
        success = self.move_to_position(goal.position, goal.max_effort)
        
        if not success:
            self.logger.error("Failed to start gripper movement")
            goal_handle.abort()
            return self._create_result(False, True)
        
        # Wait for movement completion
        start_time = time.time()
        timeout = 8.0 if self.simulation_mode else 15.0  # Longer timeout for hardware
        
        while self.is_moving and (time.time() - start_time < timeout):
            if not goal_handle.is_active:
                self.logger.info("Goal was cancelled")
                return GripperCommand.Result()
            
            # Provide feedback
            feedback = self._create_feedback()
            goal_handle.publish_feedback(feedback)
            
            # Small delay to prevent excessive CPU usage
            time.sleep(0.1)
        
        # Check result
        movement_success = not self.is_moving
        stalled = not movement_success
        
        if stalled:
            self.logger.warning(f"Gripper movement timed out after {timeout} seconds")
        
        result = self._create_result(movement_success, stalled)
        
        if movement_success:
            goal_handle.succeed()
            self.logger.info("Gripper action completed successfully")
        else:
            goal_handle.abort()
            self.logger.error("Gripper action failed")
        
        return result
    
    def _create_feedback(self):
        """Create feedback message"""
        feedback = GripperCommand.Feedback()
        feedback.position = self.current_position
        feedback.effort = self.current_force
        feedback.reached_goal = False
        feedback.stalled = False
        return feedback
    
    def _create_result(self, reached_goal, stalled):
        """Create result message"""
        result = GripperCommand.Result()
        result.position = self.current_position
        result.effort = self.current_force
        result.reached_goal = reached_goal
        result.stalled = stalled
        return result
    
    def move_to_position(self, position: float, force: float = 50.0) -> bool:
        """Move gripper to specified position"""
        with self.lock:
            # Validate inputs
            position = max(self.min_width, min(self.max_width, position))
            force = min(self.max_force, max(0.0, force))
            
            self.logger.info(f"Moving gripper to: {position:.3f}m, force: {force:.1f}%")
            
            if not self.simulation_mode and self.client and self.client.is_connected:
                # Hardware mode
                pos_units = self.meters_to_units(position)
                force_units = self.force_to_units(force)
                success = self.send_gripper_command(pos_units, force_units)
                if success:
                    self.target_position = position
                    self.is_moving = True
            else:
                # Simulation mode
                success = True
                self.simulate_movement(position, force)
            
            return success
    
    def send_gripper_command(self, position: int, force: int) -> bool:
        """Send command to real hardware"""
        if not self.client or not self.client.is_connected:
            self.logger.error("No connection to gripper hardware")
            return False
        
        try:
            # OnRobot command structure for 2FG7 gripper
            command = self.create_command_bytes(position, force)
            response = self.client.send_command(command, expect_response=True)
            
            if response and len(response) > 0:
                self.logger.debug("Command sent successfully to hardware")
                return True
            else:
                self.logger.error("No response from gripper hardware")
                return False
                
        except Exception as e:
            self.logger.error(f"Error sending gripper command: {e}")
            return False
    
    def create_command_bytes(self, position: int, force: int) -> bytes:
        """Create command bytes based on OnRobot protocol for 2FG7"""
        # This is a simplified version - adjust based on actual OnRobot protocol
        command = bytearray(8)
        
        # Command header for grip command
        command[0] = 0x01  # Grip command
        command[1] = 0x00
        
        # Position (2 bytes)
        command[2] = position & 0xFF
        command[3] = (position >> 8) & 0xFF
        
        # Speed (fixed for now, adjust as needed)
        command[4] = 0xFF  # Maximum speed
        command[5] = 0x00
        
        # Force (2 bytes)
        command[6] = force & 0xFF
        command[7] = (force >> 8) & 0xFF
        
        return bytes(command)
    
    def read_gripper_status(self) -> bool:
        """Read current gripper status"""
        if self.simulation_mode or not self.client or not self.client.is_connected:
            # For simulation, just return current state
            self.is_ready = True
            return True
        
        try:
            # Read status registers from gripper
            # Address and count depend on OnRobot protocol
            status_data = self.client.read_holding_registers(0x0000, 3)
            
            if status_data:
                self.parse_status_data(status_data)
                return True
            else:
                self.logger.warning("No status data received from gripper")
                return False
                
        except Exception as e:
            self.logger.error(f"Error reading gripper status: {e}")
            return False
    
    def parse_status_data(self, data: list):
        """Parse status data from gripper registers"""
        if len(data) >= 3:
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
        else:
            self.logger.warning(f"Invalid status data length: {len(data)}")
    
    def simulate_movement(self, target_position: float, force: float = 50.0):
        """Simulate gripper movement for simulation mode"""
        def movement_thread():
            self.is_moving = True
            self.target_position = target_position
            
            steps = 20  # Number of simulation steps
            start_position = self.current_position
            step_size = (target_position - start_position) / steps
            
            for i in range(steps):
                if not self.is_moving:  # Allow cancellation
                    break
                    
                # Update position
                self.current_position = start_position + (i + 1) * step_size
                
                # Simulate force (decreases as we approach target)
                progress = (i + 1) / steps
                self.current_force = force * (1.0 - progress * 0.8)  # Force reduces to 20% at end
                
                time.sleep(0.1)  # Simulate movement time
            
            # Ensure we end at exact target
            self.current_position = target_position
            self.current_force = 0.0
            self.is_moving = False
            
            self.logger.info(f"Simulated movement completed: {target_position:.3f}m")
        
        thread = threading.Thread(target=movement_thread)
        thread.daemon = True
        thread.start()
    
    def meters_to_units(self, meters: float) -> int:
        """Convert meters to gripper units"""
        if self.max_width == self.min_width:
            return 0
        normalized = (meters - self.min_width) / (self.max_width - self.min_width)
        return int(normalized * 255)
    
    def units_to_meters(self, units: int) -> float:
        """Convert gripper units to meters"""
        normalized = units / 255.0
        return self.min_width + normalized * (self.max_width - self.min_width)
    
    def force_to_units(self, force: float) -> int:
        """Convert force to gripper units"""
        normalized = force / self.max_force
        return int(normalized * 255)
    
    def units_to_force(self, units: int) -> float:
        """Convert gripper units to force"""
        normalized = units / 255.0
        return normalized * self.max_force
    
    def timer_callback(self):
        """Timer callback for periodic status updates"""
        self.publish_status()
    
    def publish_status(self):
        """Publish current gripper status"""
        # Read actual status from gripper or simulate
        if not self.read_gripper_status():
            # If status read failed, use current state
            pass
        
        # # Publish joint state - UPDATED to match URDF joint names
        # joint_state = JointState()
        # joint_state.header.stamp = self.node.get_clock().now().to_msg()
        # joint_state.name = ['left_finger_joint', 'right_finger_joint']
        # joint_state.position = [self.current_position / 2, self.current_position / 2]  # Split between fingers
        # joint_state.velocity = [0.0, 0.0]
        # joint_state.effort = [self.current_force, self.current_force]
        
        # self.joint_state_pub.publish(joint_state)
        
        # Publish status
        status_msg = Bool()
        status_msg.data = self.is_ready
        self.status_pub.publish(status_msg)
        
        # Publish position
        position_msg = Float32()
        position_msg.data = self.current_position
        self.position_pub.publish(position_msg)
    
    def disconnect(self):
        """Cleanup and disconnect from hardware"""
        self.is_moving = False  # Stop any ongoing movement
        
        if self.client and self.client.is_connected:
            self.client.disconnect()
            self.logger.info("Disconnected from OnRobot Compute Box")