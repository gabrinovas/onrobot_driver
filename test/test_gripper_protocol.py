#!/usr/bin/env python3
"""
Diagnostic tool for OnRobot 2FG7 gripper protocol
"""
import socket
import struct
import time

def test_modbus_commands():
    print("🔧 OnRobot 2FG7 Protocol Diagnostic")
    print("=" * 50)
    
    try:
        # Connect to Compute Box
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5.0)
        sock.connect(('192.168.1.1', 502))
        print("✅ Connected to Compute Box")
        
        # Test different Modbus function codes and register addresses
        
        # Test 1: Read Input Registers (function 0x04) - Common for status
        print("\n1. Testing Read Input Registers (0x04)...")
        for start_addr in [0x0000, 0x1000, 0x2000, 0x3000]:
            frame = struct.pack('>HHHBBHH', 
                              1, 0, 6, 1, 0x04, start_addr, 10)
            sock.send(frame)
            try:
                response = sock.recv(1024)
                if response:
                    print(f"  Addr 0x{start_addr:04X}: Got {len(response)} bytes")
                    print(f"    Raw: {response.hex()}")
                else:
                    print(f"  Addr 0x{start_addr:04X}: No response")
            except socket.timeout:
                print(f"  Addr 0x{start_addr:04X}: Timeout")
        
        # Test 2: Read Holding Registers (function 0x03)
        print("\n2. Testing Read Holding Registers (0x03)...")
        for start_addr in [0x0000, 0x1000, 0x2000, 0x3000]:
            frame = struct.pack('>HHHBBHH', 
                              2, 0, 6, 1, 0x03, start_addr, 10)
            sock.send(frame)
            try:
                response = sock.recv(1024)
                if response:
                    print(f"  Addr 0x{start_addr:04X}: Got {len(response)} bytes")
                    print(f"    Raw: {response.hex()}")
                else:
                    print(f"  Addr 0x{start_addr:04X}: No response")
            except socket.timeout:
                print(f"  Addr 0x{start_addr:04X}: Timeout")
        
        # Test 3: Try common OnRobot command structure
        print("\n3. Testing Common OnRobot Commands...")
        
        # Command to open gripper (typical structure)
        commands = [
            bytes([0x01, 0x10, 0x00, 0x00, 0x00, 0x02, 0x04, 0x00, 0x00, 0x03, 0xE8]),  # Open
            bytes([0x01, 0x10, 0x00, 0x00, 0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00]),  # Close
            bytes([0x01, 0x05, 0x00, 0x00, 0xFF, 0x00]),  # Enable
            bytes([0x01, 0x05, 0x00, 0x01, 0x00, 0x00]),  # Disable
        ]
        
        for i, cmd in enumerate(commands):
            # Add Modbus TCP header
            header = struct.pack('>HHHBB', 3+i, 0, len(cmd), 1, 0)
            full_cmd = header + cmd
            sock.send(full_cmd)
            try:
                response = sock.recv(1024)
                if response:
                    print(f"  Command {i}: Got {len(response)} bytes")
                    print(f"    Response: {response.hex()}")
                else:
                    print(f"  Command {i}: No response")
            except socket.timeout:
                print(f"  Command {i}: Timeout")
        
        sock.close()
        
    except Exception as e:
        print(f"❌ Connection failed: {e}")
        return False
    
    return True

if __name__ == '__main__':
    success = test_modbus_commands()
    exit(0 if success else 1)