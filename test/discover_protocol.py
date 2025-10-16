#!/usr/bin/env python3
"""
Discover the correct protocol for OnRobot 2FG7
"""
import socket
import struct
import time

def test_protocols():
    print("🔍 OnRobot 2FG7 Protocol Discovery")
    print("=" * 50)
    
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(2.0)
        sock.connect(('192.168.1.1', 502))
        print("✅ Connected to Compute Box")
        
        # Test various command formats that OnRobot might use
        
        commands = [
            # Simple commands
            bytes([0x00]),  # Null command
            bytes([0x01]),  # Simple command 1
            bytes([0x02]),  # Simple command 2
            bytes([0x03]),  # Status read
            
            # 8-byte commands (common in industrial protocols)
            bytes([0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]),  # Command 1
            bytes([0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]),  # Command 2  
            bytes([0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]),  # Status
            bytes([0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]),  # Command 4
            
            # With some data
            bytes([0x01, 0x00, 0x80, 0x00, 0xFF, 0x00, 0x64, 0x00]),  # Move command
            bytes([0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01]),  # Status with param
            
            # ASCII commands (some devices use text protocols)
            b"STATUS\r\n",
            b"GET POS\r\n", 
            b"VERSION\r\n",
        ]
        
        for i, cmd in enumerate(commands):
            print(f"\nTesting command {i}: {cmd.hex() if len(cmd) < 20 else '...'}")
            try:
                sock.send(cmd)
                response = sock.recv(1024)
                if response:
                    print(f"  ✅ RESPONSE: {len(response)} bytes")
                    print(f"     Hex: {response.hex()}")
                    if all(32 <= b < 127 for b in response):
                        print(f"     ASCII: {response.decode('ascii', errors='replace')}")
                else:
                    print(f"  ⚠️  No response")
            except socket.timeout:
                print(f"  ⏰ Timeout")
            except ConnectionResetError:
                print(f"  🔌 Connection reset")
                # Reconnect
                sock.close()
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(2.0)
                sock.connect(('192.168.1.1', 502))
                print("  Reconnected to Compute Box")
            except Exception as e:
                print(f"  ❌ Error: {e}")
        
        sock.close()
        
    except Exception as e:
        print(f"❌ Connection failed: {e}")
        return False
    
    return True

if __name__ == '__main__':
    success = test_protocols()
    exit(0 if success else 1)