import socket
import struct
import time
from typing import Optional, List
import threading

class OnRobotTCPClient:
    """
    Correct TCP client for OnRobot 2FG7 grippers.
    Based on OnRobot's proprietary protocol, not Modbus.
    """
    
    def __init__(self, ip: str = "192.168.1.1", port: int = 502, timeout: float = 5.0):
        self.ip = ip
        self.port = port
        self.timeout = timeout
        self.socket: Optional[socket.socket] = None
        self.is_connected = False
        self.lock = threading.Lock()
        
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
        """
        Send command to OnRobot gripper and receive response.
        OnRobot uses a simple TCP protocol, not Modbus.
        """
        if not self.is_connected:
            return None
        
        with self.lock:
            try:
                self.socket.sendall(command)
                
                if expect_response:
                    # OnRobot responses are typically small (8-16 bytes)
                    response = self.socket.recv(1024)
                    return response
                return None
                
            except socket.timeout:
                print("Error sending command: timed out")
                return None
            except Exception as e:
                print(f"Error sending command: {e}")
                self.is_connected = False
                return None
    
    def read_gripper_status(self) -> Optional[bytes]:
        """
        Read gripper status using OnRobot's protocol.
        Command: 0x03 - Read gripper status
        """
        # Status read command for OnRobot grippers
        status_cmd = bytes([0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        return self.send_command(status_cmd)
    
    def move_gripper(self, position: int, speed: int = 255, force: int = 100) -> bool:
        """
        Move gripper to specified position.
        Command: 0x01 - Grip command
        
        Args:
            position: 0-255 (0=closed, 255=open)
            speed: 0-255 (0=slow, 255=fast)
            force: 0-100 (0=min, 100=max)
        """
        # Convert force percentage to 0-255 range
        force_byte = int(force * 2.55)
        
        # Grip command structure for OnRobot
        grip_cmd = bytes([
            0x01,           # Command: Grip
            0x00,           # Reserved
            position & 0xFF, # Position low byte
            (position >> 8) & 0xFF, # Position high byte
            speed & 0xFF,   # Speed
            (speed >> 8) & 0xFF, # Speed high byte
            force_byte & 0xFF, # Force
            (force_byte >> 8) & 0xFF  # Force high byte
        ])
        
        response = self.send_command(grip_cmd)
        return response is not None and len(response) > 0
    
    def stop_gripper(self) -> bool:
        """
        Stop gripper movement.
        Command: 0x02 - Stop command
        """
        stop_cmd = bytes([0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        response = self.send_command(stop_cmd)
        return response is not None and len(response) > 0
    
    def parse_status_response(self, response: bytes) -> dict:
        """
        Parse status response from OnRobot gripper.
        Response format: [status, position_low, position_high, force_low, force_high, ...]
        """
        if len(response) < 8:
            return {}
            
        status = {
            'ready': bool(response[0] & 0x01),
            'moving': bool(response[0] & 0x02),
            'object_detected': bool(response[0] & 0x04),
            'position': (response[2] << 8) | response[1],  # Little-endian
            'force': (response[4] << 8) | response[3],     # Little-endian
            'raw_response': response.hex()
        }
        return status