# onrobot_driver/drivers/ethercat_client.py
import socket
import struct
import time
from typing import Optional

class EthercatClient:
    """
    Simple TCP client for OnRobot Compute Box communication
    Uses standard socket library - no external dependencies needed
    """
    
    def __init__(self, ip: str = "192.168.1.1", port: int = 502, timeout: float = 2.0):
        self.ip = ip
        self.port = port
        self.timeout = timeout
        self.socket: Optional[socket.socket] = None
        self.is_connected = False
        
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
    
    def send_command(self, command: bytes, expect_response: bool = True) -> Optional[bytes]:
        """Send command and receive response"""
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
        """Read status from gripper"""
        # Modify this based on OnRobot's specific status reading protocol
        status_command = b'\x00\x01'  # Example status command
        return self.send_command(status_command, expect_response=True)
    
    def is_ready(self) -> bool:
        """Check if gripper is ready"""
        status = self.read_status()
        if status and len(status) > 0:
            # Parse status based on OnRobot protocol
            # This is a placeholder - adjust based on actual protocol
            return status[0] & 0x01 != 0
        return False

class ModbusTCPClient(EthercatClient):
    """
    Modbus TCP implementation for OnRobot grippers
    Many industrial devices use Modbus TCP protocol
    """
    
    def __init__(self, ip: str = "192.168.1.1", port: int = 502, unit_id: int = 1):
        super().__init__(ip, port)
        self.unit_id = unit_id
        self.transaction_id = 0
        
    def _create_modbus_frame(self, function_code: int, data: bytes) -> bytes:
        """Create Modbus TCP frame"""
        self.transaction_id = (self.transaction_id + 1) % 65536
        length = len(data) + 2  # +2 for unit_id and function_code
        
        frame = struct.pack('>HHHB', 
                           self.transaction_id,  # Transaction ID
                           0x0000,              # Protocol ID
                           length,              # Length
                           self.unit_id)        # Unit ID
        
        frame += bytes([function_code]) + data
        return frame
    
    def read_holding_registers(self, address: int, count: int) -> Optional[list]:
        """Read holding registers"""
        data = struct.pack('>HH', address, count)
        frame = self._create_modbus_frame(0x03, data)
        
        response = self.send_command(frame, expect_response=True)
        if response and len(response) >= 9:
            # Parse response
            byte_count = response[8]
            if len(response) >= 9 + byte_count:
                values = []
                for i in range(0, byte_count, 2):
                    if i + 1 < byte_count:
                        value = struct.unpack('>H', response[9+i:9+i+2])[0]
                        values.append(value)
                return values
        return None
    
    def write_single_register(self, address: int, value: int) -> bool:
        """Write single register"""
        data = struct.pack('>HH', address, value)
        frame = self._create_modbus_frame(0x06, data)
        
        response = self.send_command(frame, expect_response=True)
        return response is not None and len(response) >= 8