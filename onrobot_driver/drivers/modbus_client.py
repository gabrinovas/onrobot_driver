import socket
import struct
import time
from typing import Optional, List
import threading

class ModbusTCPClient:
    """
    Corrected Modbus TCP client for OnRobot Compute Box with PNP configuration.
    Based on actual OnRobot 2FG7 Modbus protocol for PNP mode.
    """
    
    # OnRobot 2FG7 PNP Mode register mappings
    # These are the most common addresses for PNP configuration
    ONROBOT_2FG7_REGISTERS = {
        'status': 0x0000,        # Status register (read-only)
        'control': 0x0001,       # Control register (write-only)
        'width_actual': 0x0002,  # Actual width (read-only)
        'force_actual': 0x0003,  # Actual force (read-only)
        'width_target': 0x0004,  # Target width (write-only)
        'force_target': 0x0005,  # Target force (write-only)
        'grip_detected': 0x0006, # Grip detection (read-only)
    }
    
    # Alternative register mappings for different firmware versions
    ALTERNATIVE_REGISTERS = [
        {
            'status': 0x1000,
            'control': 0x1001,
            'width_actual': 0x1002,
            'force_actual': 0x1003,
            'width_target': 0x1004,
            'force_target': 0x1005,
            'grip_detected': 0x1006,
        },
        {
            'status': 0x2000,
            'control': 0x2001,
            'width_actual': 0x2002,
            'force_actual': 0x2003,
            'width_target': 0x2004,
            'force_target': 0x2005,
            'grip_detected': 0x2006,
        },
        {
            'status': 0x3000,
            'control': 0x3001,
            'width_actual': 0x3002,
            'force_actual': 0x3003,
            'width_target': 0x3004,
            'force_target': 0x3005,
            'grip_detected': 0x3006,
        }
    ]
    
    def __init__(self, ip: str = "192.168.1.1", port: int = 502, unit_id: int = 1, timeout: float = 5.0):
        self.ip = ip
        self.port = port
        self.unit_id = unit_id
        self.timeout = timeout
        self.socket: Optional[socket.socket] = None
        self.is_connected = False
        self.transaction_id = 0
        self.lock = threading.Lock()
        self.current_register_map = self.ONROBOT_2FG7_REGISTERS.copy()
        self.register_map_validated = False
        
    def connect(self) -> bool:
        """Connect to the OnRobot Compute Box"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(self.timeout)
            self.socket.connect((self.ip, self.port))
            self.is_connected = True
            print(f"✅ Connected to OnRobot Compute Box at {self.ip}:{self.port}")
            
            # Try to auto-detect register mapping
            if not self.validate_register_map():
                print("⚠️ Standard register map failed, trying to auto-detect...")
                self.auto_detect_registers()
                
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
        """
        Create a proper Modbus TCP frame according to standard.
        """
        self.transaction_id = (self.transaction_id + 1) % 65536
        length = len(data) + 1  # +1 for unit_id
        
        # MBAP Header (Modbus Application Protocol)
        header = struct.pack('>HHHB', 
                           self.transaction_id,  # Transaction ID
                           0x0000,              # Protocol ID (always 0 for Modbus)
                           length,              # Length
                           self.unit_id)        # Unit ID
        
        # PDU (Protocol Data Unit)
        pdu = bytes([function_code]) + data
        
        return header + pdu
    
    def _parse_modbus_response(self, response: bytes, expected_function: int) -> Optional[List[int]]:
        """
        Parse Modbus response and validate structure.
        """
        if len(response) < 8:
            print(f"❌ Response too short: {len(response)} bytes")
            return None
            
        # Parse MBAP header
        transaction_id, protocol_id, length, unit_id = struct.unpack('>HHHB', response[:7])
        
        # Validate header
        if protocol_id != 0:
            print(f"❌ Invalid protocol ID: {protocol_id}")
            return None
            
        if unit_id != self.unit_id:
            print(f"❌ Unit ID mismatch: expected {self.unit_id}, got {unit_id}")
            return None
        
        # Parse PDU
        function_code = response[7]
        if function_code != expected_function:
            # Check for error response
            if function_code == expected_function + 0x80:
                error_code = response[8] if len(response) > 8 else 0
                print(f"❌ Modbus error response: function 0x{function_code:02X}, error code 0x{error_code:02X}")
                return None
            print(f"❌ Function code mismatch: expected 0x{expected_function:02X}, got 0x{function_code:02X}")
            return None
        
        # Handle different function codes
        if expected_function == 0x03:  # Read Holding Registers
            if len(response) < 9:
                print(f"❌ Response too short for function 0x03: {len(response)} bytes")
                return None
                
            byte_count = response[8]
            if len(response) < 9 + byte_count:
                print(f"❌ Response too short for byte_count {byte_count}: {len(response)} bytes")
                return None
                
            values = []
            for i in range(0, byte_count, 2):
                if i + 1 < byte_count:
                    value = struct.unpack('>H', response[9+i:9+i+2])[0]
                    values.append(value)
            return values
            
        elif expected_function == 0x10:  # Write Multiple Registers
            # Response should be: [transaction, protocol, length, unit_id, function, address, quantity]
            if len(response) >= 12:
                address = struct.unpack('>H', response[8:10])[0]
                quantity = struct.unpack('>H', response[10:12])[0]
                return [address, quantity]
        
        return None
    
    def read_holding_registers(self, address: int, count: int) -> Optional[List[int]]:
        """
        Read holding registers (function 0x03).
        """
        if not self.is_connected:
            return None
        
        with self.lock:
            try:
                # Create read command
                data = struct.pack('>HH', address, count)
                frame = self._create_modbus_frame(0x03, data)
                
                # Send and receive
                self.socket.sendall(frame)
                response = self.socket.recv(256)
                
                return self._parse_modbus_response(response, 0x03)
                
            except socket.timeout:
                print(f"⏰ Timeout reading holding registers at address 0x{address:04X}")
                return None
            except Exception as e:
                print(f"❌ Error reading holding registers: {e}")
                return None
    
    def write_multiple_registers(self, address: int, values: List[int]) -> bool:
        """
        Write multiple registers (function 0x10).
        Used for commanding the gripper.
        """
        if not self.is_connected:
            return False
        
        with self.lock:
            try:
                # Prepare data
                byte_count = len(values) * 2
                data = struct.pack('>HHB', address, len(values), byte_count)
                
                # Add register values
                for value in values:
                    data += struct.pack('>H', value)
                
                frame = self._create_modbus_frame(0x10, data)
                
                # Send and receive
                self.socket.sendall(frame)
                response = self.socket.recv(256)
                
                result = self._parse_modbus_response(response, 0x10)
                return result is not None
                
            except socket.timeout:
                print(f"⏰ Timeout writing registers at address 0x{address:04X}")
                return False
            except Exception as e:
                print(f"❌ Error writing registers: {e}")
                return False
    
    def write_single_register(self, address: int, value: int) -> bool:
        """
        Write single register (function 0x06).
        """
        if not self.is_connected:
            return False
        
        with self.lock:
            try:
                data = struct.pack('>HH', address, value)
                frame = self._create_modbus_frame(0x06, data)
                
                self.socket.sendall(frame)
                response = self.socket.recv(256)
                
                # For function 0x06, response should echo the request
                return len(response) >= 8
                
            except socket.timeout:
                print(f"⏰ Timeout writing single register at address 0x{address:04X}")
                return False
            except Exception as e:
                print(f"❌ Error writing single register: {e}")
                return False

    def read_gripper_status(self) -> Optional[dict]:
        """
        Read all gripper status registers at once.
        Returns dict with status, width_actual, force_actual.
        """
        try:
            # Try to read all status registers in one go
            result = self.read_holding_registers(
                self.current_register_map['status'], 
                4  # Read status, width, force, grip_detected
            )
            
            if result and len(result) >= 3:
                return {
                    'status': result[0],
                    'width_actual': result[1],
                    'force_actual': result[2],
                    'grip_detected': result[3] if len(result) > 3 else 0
                }
            
            # Fallback: try reading registers individually
            status_data = {}
            for reg_name in ['status', 'width_actual', 'force_actual', 'grip_detected']:
                if reg_name in self.current_register_map:
                    result = self.read_holding_registers(self.current_register_map[reg_name], 1)
                    if result:
                        status_data[reg_name] = result[0]
            
            return status_data if status_data else None
            
        except Exception as e:
            print(f"❌ Error reading gripper status: {e}")
            return None

    def send_gripper_command(self, width_target: int, force_target: int) -> bool:
        """
        Send gripper command to target width and force.
        """
        try:
            # Write to both target registers
            return self.write_multiple_registers(
                self.current_register_map['width_target'], 
                [width_target, force_target]
            )
        except Exception as e:
            print(f"❌ Error sending gripper command: {e}")
            return False

    def validate_register_map(self) -> bool:
        """
        Validate if the current register map is correct by reading status register.
        """
        try:
            result = self.read_holding_registers(self.current_register_map['status'], 1)
            if result is not None:
                print(f"✅ Register map validated - status register 0x{self.current_register_map['status']:04X} responded")
                self.register_map_validated = True
                return True
            return False
        except Exception as e:
            print(f"❌ Register map validation failed: {e}")
            return False

    def auto_detect_registers(self):
        """
        Auto-detect the correct register mapping by scanning common address ranges.
        """
        print("🔍 Auto-detecting register mapping...")
        
        # Common address ranges to scan
        scan_ranges = [
            (0x0000, 0x0020),   # Standard PNP addresses
            (0x1000, 0x1020),   # Alternative range 1
            (0x2000, 0x2020),   # Alternative range 2
            (0x3000, 0x3020),   # Alternative range 3
            (0x4000, 0x4020),   # Alternative range 4
        ]
        
        for start_addr, end_addr in scan_ranges:
            print(f"🔍 Scanning range 0x{start_addr:04X}-0x{end_addr:04X}...")
            valid_registers = self.scan_registers(start_addr, end_addr - start_addr)
            
            if valid_registers:
                print(f"✅ Found valid registers in range 0x{start_addr:04X}")
                # Try to identify register types based on common patterns
                self.identify_registers(valid_registers)
                break
        
        if not self.register_map_validated:
            print("❌ Could not auto-detect register mapping")
        else:
            print("✅ Register mapping auto-detection completed")

    def identify_registers(self, valid_registers: dict):
        """
        Identify register types based on common patterns and values.
        """
        print("🔍 Identifying register types...")
        
        # Common patterns for identification
        for addr, value in valid_registers.items():
            if value in [0, 1, 2, 3]:  # Status register often has small values
                if 'status' not in self.current_register_map:
                    self.current_register_map['status'] = addr
                    print(f"  ✅ Identified status register at 0x{addr:04X}")
            
            elif 0 <= value <= 255:  # Width/force values are typically 0-255
                if 'width_actual' not in self.current_register_map:
                    self.current_register_map['width_actual'] = addr
                    print(f"  ✅ Identified width register at 0x{addr:04X}")
                elif 'force_actual' not in self.current_register_map:
                    self.current_register_map['force_actual'] = addr
                    print(f"  ✅ Identified force register at 0x{addr:04X}")
        
        # Validate the identified map
        if self.validate_register_map():
            print("✅ Register identification successful")
        else:
            print("❌ Register identification failed")

    def scan_registers(self, start_addr: int = 0, count: int = 10) -> dict:
        """
        Scan registers to find valid addresses for OnRobot gripper.
        """
        print(f"🔍 Scanning registers from 0x{start_addr:04X} to 0x{start_addr+count-1:04X}...")
        
        valid_registers = {}
        
        for addr in range(start_addr, start_addr + count):
            result = self.read_holding_registers(addr, 1)
            if result is not None:
                valid_registers[addr] = result[0]
                print(f"✅ Register 0x{addr:04X}: {result[0]} (0x{result[0]:04X})")
            # else:
            #     print(f"❌ Register 0x{addr:04X}: No response")
            
            time.sleep(0.05)  # Small delay to not overwhelm the device
        
        return valid_registers

    def get_register_info(self) -> dict:
        """
        Get information about the current register mapping.
        """
        return {
            'current_map': self.current_register_map,
            'validated': self.register_map_validated,
            'connected': self.is_connected
        }