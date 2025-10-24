#!/usr/bin/env python3

import socket
import struct
import time
import sys

class RegisterDetector:
    def __init__(self, ip="192.168.1.1", port=502):
        self.ip = ip
        self.port = port
        self.socket = None
        
    def connect(self):
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(3.0)
            self.socket.connect((self.ip, self.port))
            print(f"✅ Connected to {self.ip}:{self.port}")
            return True
        except Exception as e:
            print(f"❌ Connection failed: {e}")
            return False
    
    def create_modbus_frame(self, function_code, data, unit_id=1):
        transaction_id = 1
        length = len(data) + 1
        header = struct.pack('>HHHB', transaction_id, 0x0000, length, unit_id)
        pdu = bytes([function_code]) + data
        return header + pdu
    
    def parse_response(self, response, expected_function):
        if len(response) < 8:
            return None
            
        transaction_id, protocol_id, length, unit_id = struct.unpack('>HHHB', response[:7])
        
        if protocol_id != 0:
            return None
            
        function_code = response[7]
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
        
        return None
    
    def read_register(self, address, count=1):
        try:
            data = struct.pack('>HH', address, count)
            frame = self.create_modbus_frame(0x03, data)
            
            self.socket.sendall(frame)
            response = self.socket.recv(256)
            
            return self.parse_response(response, 0x03)
        except socket.timeout:
            return None
        except Exception as e:
            return None
    
    def scan_range(self, start_addr, count, description=""):
        print(f"\n🔍 Scanning {description} (0x{start_addr:04X}-0x{start_addr+count-1:04X})...")
        found = []
        
        for addr in range(start_addr, start_addr + count):
            result = self.read_register(addr, 1)
            if result is not None:
                found.append((addr, result[0]))
                print(f"✅ Register 0x{addr:04X}: {result[0]} (0x{result[0]:04X})")
            # Uncomment for verbose scanning
            # else:
            #     print(f"❌ Register 0x{addr:04X}: No response")
            
            time.sleep(0.02)  # Small delay
        
        return found
    
    def comprehensive_scan(self):
        """Perform comprehensive scan of all possible register ranges"""
        if not self.connect():
            return
        
        all_found = []
        
        # Common OnRobot address ranges
        scan_ranges = [
            (0x0000, 32, "Standard PNP Range 1"),
            (0x1000, 32, "Alternative Range 1"), 
            (0x2000, 32, "Alternative Range 2"),
            (0x3000, 32, "Alternative Range 3"),
            (0x4000, 32, "Alternative Range 4"),
            (0x5000, 32, "Alternative Range 5"),
            (0x6000, 32, "Alternative Range 6"),
            (0x7000, 32, "Alternative Range 7"),
            (0x8000, 32, "Alternative Range 8"),
            (0x9000, 32, "Alternative Range 9"),
            (0xA000, 32, "Alternative Range 10"),
            (0xB000, 32, "Alternative Range 11"),
            (0xC000, 32, "Alternative Range 12"),
            (0xD000, 32, "Alternative Range 13"),
            (0xE000, 32, "Alternative Range 14"),
            (0xF000, 32, "Alternative Range 15"),
        ]
        
        for start_addr, count, description in scan_ranges:
            found = self.scan_range(start_addr, count, description)
            all_found.extend(found)
            
            if found:
                print(f"🎯 Found {len(found)} registers in {description}")
        
        # Special known OnRobot addresses
        special_addresses = [
            (0x07D0, "Legacy Status"),
            (0x07D1, "Legacy Width"), 
            (0x07D2, "Legacy Force"),
            (0x07D3, "Legacy Target Width"),
            (0x07D4, "Legacy Target Force"),
            (0x07D5, "Legacy Control"),
        ]
        
        print(f"\n🔍 Scanning known special addresses...")
        for addr, desc in special_addresses:
            result = self.read_register(addr, 1)
            if result is not None:
                print(f"✅ Special {desc} 0x{addr:04X}: {result[0]}")
                all_found.append((addr, result[0]))
        
        print(f"\n📊 SCAN SUMMARY:")
        print(f"Total registers found: {len(all_found)}")
        
        if all_found:
            print("\n📋 Found registers:")
            for addr, value in sorted(all_found):
                print(f"  0x{addr:04X}: {value} (0x{value:04X})")
            
            # Try to identify register types
            self.identify_registers(all_found)
        else:
            print("❌ No registers found in any range!")
            
            # Test if Modbus is responding at all
            print("\n🔍 Testing basic Modbus communication...")
            test_result = self.read_register(0x0000, 1)
            if test_result is None:
                print("❌ Modbus not responding at all - check connection/configuration")
            else:
                print("✅ Modbus is responding but no gripper registers found")
        
        self.socket.close()
        return all_found
    
    def identify_registers(self, registers):
        """Try to identify what each register does"""
        print(f"\n🔍 Identifying register functions...")
        
        reg_map = {}
        
        for addr, value in registers:
            if value == 0 or value == 1:
                reg_map[addr] = "Status/Control"
            elif 0 <= value <= 255:
                if 'width' not in [v for v in reg_map.values()]:
                    reg_map[addr] = "Width"
                elif 'force' not in [v for v in reg_map.values()]:
                    reg_map[addr] = "Force"
                else:
                    reg_map[addr] = "Data"
            else:
                reg_map[addr] = "Unknown"
        
        print("📋 Register identification:")
        for addr, value in registers:
            func = reg_map.get(addr, "Unknown")
            print(f"  0x{addr:04X}: {value} - {func}")

def main():
    print("🎯 OnRobot 2FG7 Comprehensive Register Detection")
    print("===============================================")
    
    detector = RegisterDetector()
    detector.comprehensive_scan()

if __name__ == '__main__':
    main()