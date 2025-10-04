import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from threading import Lock
import time

from std_msgs.msg import Bool, Float32
from sensor_msgs.msg import JointState
from control_msgs.action import GripperCommand

from .modbus_client import ModbusTCPClient

class OnRobotGripper:
    """
    Production OnRobot gripper controller - REAL HARDWARE ONLY
    No simulation fallback - fails if hardware not available
    """
    
    def __init__(self, node: Node):
        self.node = node
        self.logger = node.get_logger()
        
        # Get parameters
        self.gripper_type = self.node.get_parameter('gripper_type').value
        self.ip_address = self.node.get_parameter('ip_address').value
        self.port = self.node.get_parameter('port').value
        self.max_width = self.node.get_parameter('max_width').value
        self.min_width = self.node.get_parameter('min_width').value
        self.max_force = self.node.get_parameter('max_force').value
        self.update_rate = self.node.get_parameter('update_rate').value
        
        # Gripper state
        self.current_position = 0.0
        self.current_force = 0.0
        self.is_ready = False
        self.is_moving = False
        self.lock = Lock()
        
        # Communication client - REQUIRED for hardware
        self.client = None
        
        # ROS2 interfaces
        self.setup_ros_interfaces()
        
        # Setup communication - WILL FAIL IF NO HARDWARE
        self.setup_communication()
        
        self.logger.info(f"OnRobot {self.gripper_type} gripper initialized for REAL HARDWARE")
        self.logger.info(f"IP: {self.ip_address}, Port: {self.port}")
    
    def setup_communication(self):
        """Setup communication with Compute Box - HARDWARE REQUIRED"""
        self.logger.info(f"Connecting to OnRobot Compute Box at {self.ip_address}:{self.port}...")
        
        try:
            self.client = ModbusTCPClient(self.ip_address, self.port)
            
            if not self.client.connect():
                raise ConnectionError(f"Failed to connect to Compute Box at {self.ip_address}:{self.port}")
            
            self.logger.info("✅ Successfully connected to OnRobot Compute Box")
            
            # Verify gripper is responsive
            if not self.read_gripper_status():
                raise ConnectionError("Gripper not responding to status requests")
                
            self.logger.info("✅ Gripper is responsive and ready")
                    
        except Exception as e:
            self.logger.error(f"❌ HARDWARE CONNECTION FAILED: {e}")
            self.logger.error("This driver requires real OnRobot hardware.")
            self.logger.error("Please check:")
            self.logger.error("1. Compute Box is powered on and connected to network")
            self.logger.error("2. IP address is correct (default: 192.168.1.1)")
            self.logger.error("3. Network connectivity to Compute Box")
            raise RuntimeError(f"Cannot initialize OnRobot gripper: {e}")
    
    def setup_ros_interfaces(self):
        """Setup ROS2 publishers, subscribers, and action servers"""
        # Publishers
        self.joint_state_pub = self.node.create_publisher(
            JointState, 'joint_states', 10)
        self.status_pub = self.node.create_publisher(
            Bool, 'gripper_status', 10)
        self.position_pub = self.node.create_publisher(
            Float32, 'gripper_position', 10)
        
        # Action server for gripper commands
        self.action_server = ActionServer(
            self.node,
            GripperCommand,
            'gripper_action',
            self.execute_action_callback)
        
        # Timer for periodic updates
        self.timer = self.node.create_timer(1.0/self.update_rate, self.timer_callback)
    
    def timer_callback(self):
        """Timer callback for periodic status updates from hardware"""
        self.publish_status()
    
    def execute_action_callback(self, goal_handle):
        """Execute gripper action command - REAL HARDWARE ONLY"""
        goal = goal_handle.request.command
        self.logger.info(f"Executing gripper command: position={goal.position:.3f}m, force={goal.max_effort:.1f}%")
        
        # Send command to hardware
        success = self.move_to_position(goal.position, goal.max_effort)
        
        if not success:
            goal_handle.abort()
            result = GripperCommand.Result()
            result.position = self.current_position
            result.effort = self.current_force
            result.reached_goal = False
            result.stalled = True
            return result
        
        # Wait for movement completion with timeout
        start_time = time.time()
        movement_timeout = 10.0  # 10 second timeout for hardware
        
        while self.is_moving and (time.time() - start_time < movement_timeout):
            if not goal_handle.is_active:
                self.logger.info("Goal preempted")
                return GripperCommand.Result()
            
            # Provide feedback
            feedback_msg = GripperCommand.Feedback()
            feedback_msg.position = self.current_position
            feedback_msg.effort = self.current_force
            feedback_msg.reached_goal = False
            feedback_msg.stalled = False
            goal_handle.publish_feedback(feedback_msg)
            
            time.sleep(0.1)
        
        # Check if movement completed successfully
        if self.is_moving:
            self.logger.warning("Gripper movement timeout")
            success = False
        
        # Set final result
        result = GripperCommand.Result()
        result.position = self.current_position
        result.effort = self.current_force
        result.reached_goal = success
        result.stalled = not success
        
        if success:
            goal_handle.succeed()
            self.logger.info("Gripper action completed successfully")
        else:
            goal_handle.abort()
            self.logger.error("Gripper action failed")
        
        return result
    
    def move_to_position(self, position: float, force: float = 50.0) -> bool:
        """Move gripper to specified position with given force - REAL HARDWARE ONLY"""
        with self.lock:
            # Validate inputs
            position = max(self.min_width, min(self.max_width, position))
            force = min(self.max_force, max(0.0, force))
            
            self.logger.info(f"Moving gripper to: {position:.3f}m, force: {force:.1f}%")
            
            # Convert to gripper units
            pos_units = self.meters_to_units(position)
            force_units = self.force_to_units(force)
            
            # Send command to hardware
            success = self.send_gripper_command(pos_units, force_units)
            
            if not success:
                self.logger.error("Failed to send command to gripper hardware")
                return False
            
            self.is_moving = True
            return True
    
    def send_gripper_command(self, position: int, force: int) -> bool:
        """Send command to gripper via Compute Box"""
        if not self.client or not self.client.is_connected:
            self.logger.error("No connection to gripper hardware")
            return False
        
        try:
            # OnRobot command structure for 2FG7 gripper
            # This should be based on the actual OnRobot Modbus protocol
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
        # This is a simplified version - you'll need the actual protocol
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
        """Read current gripper status from hardware"""
        if not self.client or not self.client.is_connected:
            self.logger.error("Cannot read status - no hardware connection")
            return False
        
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
    
    def publish_status(self):
        """Publish current gripper status from hardware"""
        # Read actual status from hardware
        if not self.read_gripper_status():
            self.logger.warning("Failed to read gripper status")
            return
        
        # Publish joint state
        joint_state = JointState()
        joint_state.header.stamp = self.node.get_clock().now().to_msg()
        joint_state.name = ['onrobot_gripper_finger1_joint', 'onrobot_gripper_finger2_joint']
        joint_state.position = [self.current_position, self.current_position]
        joint_state.velocity = [0.0, 0.0]  # Would need to calculate from position changes
        joint_state.effort = [self.current_force, self.current_force]
        
        self.joint_state_pub.publish(joint_state)
        
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
        if self.client:
            self.client.disconnect()
            self.logger.info("Disconnected from OnRobot Compute Box")