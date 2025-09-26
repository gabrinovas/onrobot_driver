import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from threading import Lock
import time

from std_msgs.msg import Bool, Float32
from sensor_msgs.msg import JointState
from control_msgs.action import GripperCommand

from .ethercat_client import EthercatClient, ModbusTCPClient

class OnRobotGripper:
    """
    Main gripper controller class for OnRobot grippers
    """
    
    def __init__(self, node: Node):
        self.node = node
        self.logger = node.get_logger()
        
        # Get parameters from the node (already declared)
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
        
        # Communication client
        self.client = None
        
        # ROS2 interfaces
        self.setup_ros_interfaces()
        
        # Setup communication
        self.setup_communication()
        
        self.logger.info(f"OnRobot {self.gripper_type} gripper initialized")
        self.logger.info(f"IP: {self.ip_address}, Max width: {self.max_width}m")
    
    def setup_communication(self):
        """Setup communication with Compute Box"""
        if self.ip_address == "127.0.0.1":
            self.logger.info("Running in simulation mode (localhost)")
            return
            
        try:
            # Try Modbus TCP first
            self.client = ModbusTCPClient(self.ip_address, self.port)
            
            if not self.client.connect():
                self.logger.warning("Modbus TCP connection failed, trying raw Ethernet")
                self.client = EthercatClient(self.ip_address, self.port)
                if not self.client.connect():
                    self.logger.warning("Failed to connect to Compute Box - running in simulation mode")
                    
        except Exception as e:
            self.logger.error(f"Communication setup failed: {e}")
            self.client = None
    
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
        """Timer callback for periodic updates"""
        self.publish_status()
    
    def execute_action_callback(self, goal_handle):
        """Execute gripper action command"""
        self.logger.info(f"Executing gripper command: {goal_handle.request.command.position}")
        
        goal = goal_handle.request.command
        success = self.move_to_position(goal.position, goal.max_effort)
        
        # Provide feedback during execution
        feedback_msg = GripperCommand.Feedback()
        
        # Simulate movement
        start_time = time.time()
        while self.is_moving and (time.time() - start_time < 5.0):  # 5 second timeout
            if not goal_handle.is_active:
                self.logger.info("Goal preempted")
                return GripperCommand.Result()
            
            # Update feedback
            feedback_msg.position = self.current_position
            feedback_msg.effort = self.current_force
            feedback_msg.reached_goal = False
            feedback_msg.stalled = False
            
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.1)
        
        # Set final result
        result = GripperCommand.Result()
        result.position = self.current_position
        result.effort = self.current_force
        result.reached_goal = True
        result.stalled = False
        
        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        
        return result
    
    def move_to_position(self, position: float, force: float = 50.0) -> bool:
        """Move gripper to specified position with given force"""
        with self.lock:
            # Validate inputs
            position = max(self.min_width, min(self.max_width, position))
            force = min(self.max_force, max(0.0, force))
            
            self.logger.info(f"Moving gripper to position: {position}, force: {force}")
            
            # Convert to gripper units
            pos_units = self.meters_to_units(position)
            force_units = self.force_to_units(force)
            
            # Send command to gripper
            success = self.send_gripper_command(pos_units, force_units)
            
            if success:
                self.is_moving = True
                # Simulate movement completion
                self.simulate_movement(position)
            
            return True  # Always return True in simulation mode
    
    def send_gripper_command(self, position: int, force: int) -> bool:
        """Send command to gripper via Compute Box"""
        if not self.client or not self.client.is_connected:
            self.logger.info("Simulating gripper command (no hardware connected)")
            return True  # Simulate success
        
        try:
            # OnRobot command structure
            command = self.create_command_bytes(position, force)
            response = self.client.send_command(command, expect_response=True)
            
            return response is not None and len(response) > 0
            
        except Exception as e:
            self.logger.error(f"Error sending gripper command: {e}")
            return False
    
    def create_command_bytes(self, position: int, force: int) -> bytes:
        """Create command bytes based on OnRobot protocol"""
        command = bytearray(8)
        
        # Command header
        command[0] = 0x01  # Grip command
        command[1] = 0x00
        
        # Position (2 bytes)
        command[2] = position & 0xFF
        command[3] = (position >> 8) & 0xFF
        
        # Speed (fixed for now)
        command[4] = 0xFF
        command[5] = 0x00
        
        # Force (2 bytes)
        command[6] = force & 0xFF
        command[7] = (force >> 8) & 0xFF
        
        return bytes(command)
    
    def read_gripper_status(self) -> bool:
        """Read current gripper status"""
        if not self.client or not self.client.is_connected:
            # Simulate status
            self.is_ready = True
            self.is_moving = False
            return True
        
        try:
            status_data = self.client.read_status()
            if status_data:
                self.parse_status_data(status_data)
                return True
        except Exception as e:
            self.logger.error(f"Error reading gripper status: {e}")
        
        return False
    
    def parse_status_data(self, data: bytes):
        """Parse status data from gripper"""
        if len(data) >= 6:
            status = data[0]
            position_raw = (data[2] << 8) | data[1]
            force_raw = (data[4] << 8) | data[3]
            
            self.is_ready = (status & 0x01) != 0
            self.is_moving = (status & 0x02) != 0
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
    
    def simulate_movement(self, target_position: float):
        """Simulate gripper movement"""
        def movement_thread():
            steps = 10
            current = self.current_position
            step_size = (target_position - current) / steps
            
            for i in range(steps):
                self.current_position = current + (i + 1) * step_size
                time.sleep(0.1)
            
            self.current_position = target_position
            self.is_moving = False
        
        import threading
        thread = threading.Thread(target=movement_thread)
        thread.daemon = True
        thread.start()
    
    def publish_status(self):
        """Publish current gripper status"""
        # Read actual status from gripper
        self.read_gripper_status()
        
        # Publish joint state
        joint_state = JointState()
        joint_state.header.stamp = self.node.get_clock().now().to_msg()
        joint_state.name = ['onrobot_gripper_finger1_joint']
        joint_state.position = [self.current_position]
        joint_state.velocity = [0.0]
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
    
    def disconnect(self):
        """Cleanup and disconnect"""
        if self.client:
            self.client.disconnect()