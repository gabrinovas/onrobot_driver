import socket
import struct
import time
from typing import Optional, List
import threading

class ModbusTCPClient:
    """
    Corrected Modbus TCP client specifically for OnRobot Compute Box.
    Implements OnRobot-specific register mappings and protocols.
    """
    
    def __init__(self, ip: str = "192.168.1.1", port: int = 502, unit_id: int = 1, timeout: float = 5.0):
        self.ip = ip
        self.port = port
        self.unit_id = unit_id
        self.timeout = timeout
        self.socket: Optional[socket.socket] = None
        self.is_connected = False
        self.transaction_id = 0
        self.lock = threading.Lock()
        
    def connect(self) -> bool:
        """Connect to the OnRobot Compute Box"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(self.timeout)
            self.socket.connect((self.ip, self.port))
            self.is_connected = True
            print(f"✅ Connected to OnRobot Compute Box at {self.ip}:{self.port}")
            return True
        except Exception as e:
            print(f"❌ Failed to connect to {self.ip}:{self.port} - {e}")
            self.is_connected = False
            return False
    
    def disconnect(self):
        """Disconnect from the Compute Box"""
        if self.socket:
            self.socket.close()
        self.is_connected = False
    
    def _create_modbus_frame(self, function_code: int, data: bytes) -> bytes:
        """Create proper Modbus TCP frame"""
        self.transaction_id = (self.transaction_id + 1) % 65536
        length = len(data) + 1  # +1 for unit_id
        
        header = struct.pack('>HHHB', 
                           self.transaction_id,
                           0x0000,              # Protocol ID
                           length,              # Length
                           self.unit_id)        # Unit ID
        
        pdu = bytes([function_code]) + data
        return header + pdu
    
    def _parse_modbus_response(self, response: bytes, expected_function: int) -> Optional[List[int]]:
        """Parse Modbus response with proper error handling"""
        if len(response) < 8:
            return None
            
        try:
            transaction_id, protocol_id, length, unit_id = struct.unpack('>HHHB', response[:7])
            
            if protocol_id != 0 or unit_id != self.unit_id:
                return None
            
            function_code = response[7]
            
            # Check for Modbus exception
            if function_code == expected_function + 0x80:
                exception_code = response[8] if len(response) > 8 else 0
                print(f"❌ Modbus exception {exception_code} for function {expected_function:02X}")
                return None
                
            if function_code != expected_function:
                return None
            
            if expected_function == 0x03:  # Read Holding Registers
                if len(response) < 9:
                    return None
                    
                byte_count = response[8]
                if len(response) < 9 + byte_count:
                    return None
                    
                values = []
                for i in range(0, byte_count, 2):
                    if i + 1 < byte_count:
                        value = struct.unpack('>H', response[9+i:9+i+2])[0]
                        values.append(value)
                return values
                
            elif expected_function == 0x10:  # Write Multiple Registers
                if len(response) >= 12:
                    address = struct.unpack('>H', response[8:10])[0]
                    quantity = struct.unpack('>H', response[10:12])[0]
                    return [address, quantity]
            
            return None
            
        except Exception as e:
            print(f"❌ Error parsing response: {e}")
            return None
    
    def read_holding_registers(self, address: int, count: int) -> Optional[List[int]]:
        """Read holding registers with retry logic"""
        if not self.is_connected:
            return None
        
        with self.lock:
            for attempt in range(3):
                try:
                    data = struct.pack('>HH', address, count)
                    frame = self._create_modbus_frame(0x03, data)
                    
                    self.socket.sendall(frame)
                    response = self.socket.recv(256)
                    
                    result = self._parse_modbus_response(response, 0x03)
                    if result is not None:
                        return result
                        
                except socket.timeout:
                    if attempt == 2:
                        print(f"⏰ Timeout reading registers at 0x{address:04X}")
                except Exception as e:
                    if attempt == 2:
                        print(f"❌ Error reading registers: {e}")
                
                if attempt < 2:
                    time.sleep(0.1)
            
            return None
    
    def write_multiple_registers(self, address: int, values: List[int]) -> bool:
        """Write multiple registers with retry logic"""
        if not self.is_connected:
            return False
        
        with self.lock:
            for attempt in range(3):
                try:
                    byte_count = len(values) * 2
                    data = struct.pack('>HHB', address, len(values), byte_count)
                    
                    for value in values:
                        data += struct.pack('>H', value)
                    
                    frame = self._create_modbus_frame(0x10, data)
                    
                    self.socket.sendall(frame)
                    response = self.socket.recv(256)
                    
                    result = self._parse_modbus_response(response, 0x10)
                    if result is not None:
                        return True
                        
                except socket.timeout:
                    if attempt == 2:
                        print(f"⏰ Timeout writing registers at 0x{address:04X}")
                except Exception as e:
                    if attempt == 2:
                        print(f"❌ Error writing registers: {e}")
                
                if attempt < 2:
                    time.sleep(0.1)
            
            return False

    def write_single_register(self, address: int, value: int) -> bool:
        """Write single register with retry logic"""
        if not self.is_connected:
            return False
        
        with self.lock:
            for attempt in range(3):
                try:
                    data = struct.pack('>HH', address, value)
                    frame = self._create_modbus_frame(0x06, data)
                    
                    self.socket.sendall(frame)
                    response = self.socket.recv(256)
                    
                    # For function 0x06, response should echo the request
                    if len(response) >= 8:
                        return True
                        
                except socket.timeout:
                    if attempt == 2:
                        print(f"⏰ Timeout writing single register at 0x{address:04X}")
                except Exception as e:
                    if attempt == 2:
                        print(f"❌ Error writing single register: {e}")
                
                if attempt < 2:
                    time.sleep(0.1)
            
            return False

    def scan_onrobot_registers(self) -> dict:
        """
        Scan known OnRobot register addresses to find the gripper.
        Returns dictionary of valid register addresses and their values.
        """
        print("🔍 Scanning for OnRobot gripper registers...")
        
        # Known OnRobot register addresses for different models
        onrobot_addresses = [
            # 2FG7 specific addresses
            0x0000, 0x0001, 0x0002, 0x0003,  # Status registers
            0x1000, 0x1001, 0x1002, 0x1003,  # Command registers  
            0x2000, 0x2001, 0x2002, 0x2003,  # Data registers
            0x03E8, 0x03E9, 0x03EA, 0x03EB,  # Common addresses
            0x07D0, 0x07D1, 0x07D2, 0x07D3,  # Extended addresses
        ]
        
        valid_registers = {}
        
        for addr in onrobot_addresses:
            result = self.read_holding_registers(addr, 1)
            if result is not None:
                valid_registers[addr] = result[0]
                print(f"✅ Register 0x{addr:04X}: {result[0]} (0x{result[0]:04X})")
            else:
                print(f"❌ Register 0x{addr:04X}: No response")
            
            time.sleep(0.05)  # Small delay
        
        if valid_registers:
            print(f"🎯 Found {len(valid_registers)} valid registers")
        else:
            print("❌ No valid registers found - check connection and gripper power")
            
        return valid_registers