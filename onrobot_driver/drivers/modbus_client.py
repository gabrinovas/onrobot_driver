import socket
import struct
import time
from typing import Optional, List

class ModbusTCPClient:
    """
    Corrected Modbus TCP implementation for OnRobot grippers
    """
    
    def __init__(self, ip: str = "192.168.1.1", port: int = 502, unit_id: int = 1, timeout: float = 5.0):
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
        Create a proper Modbus TCP frame.
        """
        self.transaction_id = (self.transaction_id + 1) % 65536
        length = len(data) + 1  # +1 for unit_id
        
        frame = struct.pack('>HHHB', 
                           self.transaction_id,  # Transaction ID
                           0x0000,              # Protocol ID (always 0 for Modbus)
                           length,              # Length (unit_id + data)
                           self.unit_id)        # Unit ID
        
        frame += bytes([function_code]) + data
        return frame
    
    def read_holding_registers(self, address: int, count: int) -> Optional[List[int]]:
        """
        Read holding registers from the gripper.
        """
        try:
            if not self.is_connected:
                return None
                
            data = struct.pack('>HH', address, count)
            frame = self._create_modbus_frame(0x03, data)
            
            self.socket.sendall(frame)
            response = self.socket.recv(1024)
            
            if len(response) < 9:
                print(f"Invalid response length: {len(response)}")
                return None
                
            # Parse response: [transaction, protocol, length, unit_id, function, byte_count, data...]
            if response[7] != 0x03:  # Check function code
                print(f"Invalid function code in response: {response[7]}")
                return None
                
            byte_count = response[8]
            if len(response) < 9 + byte_count:
                print(f"Response too short for byte_count {byte_count}")
                return None
                
            values = []
            for i in range(0, byte_count, 2):
                if i + 1 < byte_count:
                    value = struct.unpack('>H', response[9+i:9+i+2])[0]
                    values.append(value)
                    
            return values
            
        except socket.timeout:
            print("Error sending command: timed out")
            return None
        except Exception as e:
            print(f"Error reading holding registers: {e}")
            return None
    
    def write_single_register(self, address: int, value: int) -> bool:
        """
        Write a single register on the gripper.
        """
        try:
            if not self.is_connected:
                return False
                
            data = struct.pack('>HH', address, value)
            frame = self._create_modbus_frame(0x06, data)
            
            self.socket.sendall(frame)
            response = self.socket.recv(1024)
            
            return len(response) >= 12  # Valid write response should be 12 bytes
            
        except socket.timeout:
            print("Error sending command: timed out")
            return False
        except Exception as e:
            print(f"Error writing single register: {e}")
            return False
    
    def read_input_registers(self, address: int, count: int) -> Optional[List[int]]:
        """
        Read input registers (function 0x04) - often used for status.
        """
        try:
            if not self.is_connected:
                return None
                
            data = struct.pack('>HH', address, count)
            frame = self._create_modbus_frame(0x04, data)
            
            self.socket.sendall(frame)
            response = self.socket.recv(1024)
            
            if len(response) < 9:
                print(f"Invalid response length: {len(response)}")
                return None
                
            if response[7] != 0x04:  # Check function code
                print(f"Invalid function code in response: {response[7]}")
                return None
                
            byte_count = response[8]
            if len(response) < 9 + byte_count:
                print(f"Response too short for byte_count {byte_count}")
                return None
                
            values = []
            for i in range(0, byte_count, 2):
                if i + 1 < byte_count:
                    value = struct.unpack('>H', response[9+i:9+i+2])[0]
                    values.append(value)
                    
            return values
            
        except socket.timeout:
            print("Error sending command: timed out")
            return None
        except Exception as e:
            print(f"Error reading input registers: {e}")
            return None

    def send_raw_command(self, command: bytes) -> Optional[bytes]:
        """
        Send raw command and return response.
        """
        if not self.is_connected:
            return None
        
        try:
            self.socket.sendall(command)
            return self.socket.recv(1024)
        except socket.timeout:
            print("Error sending command: timed out")
            return None
        except Exception as e:
            print(f"Error sending raw command: {e}")
            return None