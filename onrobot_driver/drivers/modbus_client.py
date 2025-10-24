import socket
import struct
import time
from typing import Optional, List
import threading

class ModbusTCPClient:
    """
    Corrected Modbus TCP client for OnRobot 2FG7 Gripper via Compute Box.
    Based on actual OnRobot 2FG7 variables provided.
    """
    
    # CORRECTED: Actual 2FG7 register mappings based on your variables
    ONROBOT_2FG7_REGISTERS = {
        'twofg_busy': 0x07D0,           # Busy status (0=ready, 1=busy)
        'twofg_width_ext': 0x07D1,      # External width measurement
        'twofg_force': 0x07D2,          # Force measurement  
        'twofg_grip_detected': 0x07D3,  # Grip detection (0=no grip, 1=grip)
        'twofg_width_int': 0x07D4,      # Internal width (usually not needed)
        'twofg_width_tgt': 0x07D5,      # Target width (write)
        'twofg_force_tgt': 0x07D6,      # Target force (write)
        'twofg_ctrl': 0x07D7,           # Control register
    }
    
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
            print(f"❌ Function code mismatch: expected {expected_function}, got {function_code}")
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
        UPDATED: Uses the actual 2FG7 variable addresses.
        """
        try:
            # Read the actual 2FG7 status registers in one batch
            result = self.read_holding_registers(self.ONROBOT_2FG7_REGISTERS['twofg_busy'], 4)
            if result and len(result) >= 4:
                return {
                    'busy': result[0],           # twofg_busy
                    'width': result[1],          # twofg_width_ext
                    'force': result[2],          # twofg_force
                    'grip_detected': result[3]   # twofg_grip_detected
                }
            
            # Fallback: Try reading individual registers
            print("⚠️ Batch read failed, trying individual registers...")
            busy = self.read_holding_registers(self.ONROBOT_2FG7_REGISTERS['twofg_busy'], 1)
            width = self.read_holding_registers(self.ONROBOT_2FG7_REGISTERS['twofg_width_ext'], 1)
            force = self.read_holding_registers(self.ONROBOT_2FG7_REGISTERS['twofg_force'], 1)
            grip = self.read_holding_registers(self.ONROBOT_2FG7_REGISTERS['twofg_grip_detected'], 1)
            
            if busy and width and force:
                return {
                    'busy': busy[0],
                    'width': width[0],
                    'force': force[0],
                    'grip_detected': grip[0] if grip else 0
                }
                
            return None
        except Exception as e:
            print(f"❌ Error reading gripper status: {e}")
            return None

    def send_gripper_command(self, width_target: int, force_target: int) -> bool:
        """
        Send gripper command to target width and force.
        UPDATED: Uses the actual 2FG7 target registers.
        """
        try:
            # Write to target width and force registers
            return self.write_multiple_registers(
                self.ONROBOT_2FG7_REGISTERS['twofg_width_tgt'], 
                [width_target, force_target]
            )
        except Exception as e:
            print(f"❌ Error sending gripper command: {e}")
            return False

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
            else:
                print(f"❌ Register 0x{addr:04X}: No response")
            
            time.sleep(0.1)  # Small delay to not overwhelm the device
        
        return valid_registers

    def test_communication(self) -> bool:
        """
        Test basic communication with the gripper.
        """
        print("🧪 Testing gripper communication...")
        
        # Try reading the known 2FG7 registers
        test_registers = [
            (self.ONROBOT_2FG7_REGISTERS['twofg_busy'], "twofg_busy"),
            (self.ONROBOT_2FG7_REGISTERS['twofg_width_ext'], "twofg_width_ext"),
            (self.ONROBOT_2FG7_REGISTERS['twofg_force'], "twofg_force"),
        ]
        
        success_count = 0
        for addr, name in test_registers:
            result = self.read_holding_registers(addr, 1)
            if result:
                print(f"✅ {name} (0x{addr:04X}): {result[0]}")
                success_count += 1
            else:
                print(f"❌ {name} (0x{addr:04X}): No response")
        
        return success_count > 0

    def get_gripper_info(self) -> dict:
        """
        Get comprehensive gripper information for diagnostics.
        """
        info = {}
        
        # Test all known registers
        for name, addr in self.ONROBOT_2FG7_REGISTERS.items():
            result = self.read_holding_registers(addr, 1)
            if result:
                info[name] = result[0]
            else:
                info[name] = "No response"
                
        return info