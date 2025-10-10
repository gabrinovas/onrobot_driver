import socket
import struct
import time
from typing import Optional, List

class ModbusTCPClient:
    """
    Modbus TCP implementation for OnRobot grippers
    Based on OnRobot's Modbus TCP protocol
    """
    
    def __init__(self, ip: str = "192.168.1.1", port: int = 502, unit_id: int = 1, timeout: float = 2.0):
        # Initialize connection parameters
        self.ip = ip
        self.port = port
        self.unit_id = unit_id
        self.timeout = timeout
        self.socket: Optional[socket.socket] = None
        self.is_connected = False
        self.transaction_id = 0
        
    def connect(self) -> bool:
        """Connect to the OnRobot Compute Box"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(self.timeout)
            self.socket.connect((self.ip, self.port))
            self.is_connected = True
            print(f"Connected to OnRobot Compute Box at {self.ip}:{self.port}")
            return True
        except Exception as e:
            print(f"Failed to connect to {self.ip}:{self.port} - {e}")
            self.is_connected = False
            return False
    
    def disconnect(self):
        """Disconnect from the Compute Box"""
        if self.socket:
            self.socket.close()
        self.is_connected = False
    
    def _create_modbus_frame(self, function_code: int, data: bytes) -> bytes:
        """
        Create a Modbus TCP frame.
        Args:
            function_code: Modbus function code (e.g., 0x03 for read, 0x06 for write)
            data: Data payload for the frame
        Returns:
            Complete Modbus TCP frame as bytes
        """
        self.transaction_id = (self.transaction_id + 1) % 65536
        length = len(data) + 2  # +2 for unit_id and function_code
        
        frame = struct.pack('>HHHB', 
                           self.transaction_id,  # Transaction ID
                           0x0000,              # Protocol ID
                           length,              # Length
                           self.unit_id)        # Unit ID
        
        frame += bytes([function_code]) + data
        return frame
    
    def read_holding_registers(self, address: int, count: int) -> Optional[List[int]]:
        """
        Read holding registers from the gripper.
        Args:
            address: Register address to start reading from
            count: Number of registers to read
        Returns:
            List of register values or None on failure
        """
        try:
            data = struct.pack('>HH', address, count)
            frame = self._create_modbus_frame(0x03, data)
            
            response = self.send_command(frame, expect_response=True)
            if response and len(response) >= 9:
                byte_count = response[8]
                if len(response) >= 9 + byte_count:
                    values = []
                    for i in range(0, byte_count, 2):
                        if i + 1 < byte_count:
                            value = struct.unpack('>H', response[9+i:9+i+2])[0]
                            values.append(value)
                    return values
            return None
        except Exception as e:
            print(f"Error reading holding registers: {e}")
            return None
    
    def write_single_register(self, address: int, value: int) -> bool:
        """
        Write a single register on the gripper.
        Args:
            address: Register address to write to
            value: Value to write
        Returns:
            True if successful, False otherwise
        """
        try:
            data = struct.pack('>HH', address, value)
            frame = self._create_modbus_frame(0x06, data)
            
            response = self.send_command(frame, expect_response=True)
            return response is not None and len(response) >= 8
        except Exception as e:
            print(f"Error writing single register: {e}")
            return False
    
    def send_command(self, command: bytes, expect_response: bool = True) -> Optional[bytes]:
        """
        Send a command to the gripper and optionally receive a response.
        Args:
            command: Command bytes to send
            expect_response: Whether to wait for a response
        Returns:
            Response bytes or None
        """
        if not self.is_connected:
            return None
        
        try:
            self.socket.sendall(command)
            if expect_response:
                return self.socket.recv(1024)
            return None
        except Exception as e:
            print(f"Error sending command: {e}")
            self.is_connected = False
            return None

    def read_status(self) -> Optional[bytes]:
        """
        Read gripper status (placeholder for actual implementation).
        Returns:
            Simulated status bytes if connected, else None.
        """
        # This would need to be implemented based on OnRobot's specific protocol
        # For now, return simulated data
        if self.is_connected:
            # Simulate status response: [ready, position, force]
            return bytes([0x01, 0x00, 0x80, 0x00, 0x40, 0x00])  # Ready, position 128, force 64
        return None