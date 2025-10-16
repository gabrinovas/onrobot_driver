#!/usr/bin/env python3
"""
Direct test for OnRobot 2FG7 gripper with real hardware
"""
import socket
import struct
import time

def test_gripper_communication():
    print("🧪 Testing OnRobot 2FG7 Gripper Communication")
    print("=" * 50)
    
    # Test basic TCP connection
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5.0)
        sock.connect(('192.168.1.1', 502))
        print("✅ TCP Connection to Compute Box: SUCCESS")
        
        # Try to read holding registers (Modbus function 0x03)
        # Common OnRobot register addresses for status
        transaction_id = 1
        protocol_id = 0
        length = 6
        unit_id = 1
        function_code = 0x03  # Read holding registers
        start_address = 0x0000
        register_count = 10   # Read first 10 registers
        
        # Build Modbus frame
        frame = struct.pack('>HHHBBHH', 
                          transaction_id, protocol_id, length,
                          unit_id, function_code, 
                          start_address, register_count)
        
        sock.send(frame)
        
        try:
            response = sock.recv(1024)
            if response:
                print(f"✅ Gripper responded with {len(response)} bytes")
                print(f"Raw response: {response.hex()}")
                
                # Parse response
                if len(response) >= 9:
                    byte_count = response[8]
                    print(f"Byte count: {byte_count}")
                    
                    if len(response) >= 9 + byte_count:
                        values = []
                        for i in range(0, byte_count, 2):
                            if i + 1 < byte_count:
                                value = struct.unpack('>H', response[9+i:9+i+2])[0]
                                values.append(value)
                        print(f"Register values: {values}")
            else:
                print("❌ No response from gripper")
                
        except socket.timeout:
            print("❌ Gripper response timeout")
        except Exception as e:
            print(f"❌ Error reading response: {e}")
            
        sock.close()
        
    except Exception as e:
        print(f"❌ Connection failed: {e}")
        return False
    
    return True

if __name__ == '__main__':
    success = test_gripper_communication()
    exit(0 if success else 1)