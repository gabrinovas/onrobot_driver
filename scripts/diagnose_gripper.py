#!/usr/bin/env python3

import sys
import os
import time
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from onrobot_driver.drivers.modbus_client import ModbusTCPClient

def main():
    print("🔧 OnRobot 2FG7 Advanced Diagnostic Tool")
    print("=" * 60)
    
    # Create client
    client = ModbusTCPClient(ip="192.168.1.1", port=502)
    
    if not client.connect():
        print("❌ Failed to connect to gripper")
        return 1
    
    print("✅ Connected to gripper")
    
    # Test communication
    if not client.test_communication():
        print("❌ Basic communication test failed")
        print("\n🔍 Let's scan for working registers...")
        
        # Scan around the known 2FG7 area
        print("\nScanning 0x07C0 to 0x07E0...")
        client.scan_registers(0x07C0, 0x20)
        
        print("\nScanning 0x0000 to 0x0020...")
        client.scan_registers(0x0000, 0x20)
        
        client.disconnect()
        return 1
    
    print("✅ Basic communication test passed")
    
    # Get comprehensive info
    print("\n📊 Gripper Information:")
    print("-" * 40)
    
    info = client.get_gripper_info()
    for name, value in info.items():
        print(f"{name:20}: {value}")
    
    # Test status reading
    print("\n🧪 Testing status reading:")
    print("-" * 40)
    
    status = client.read_gripper_status()
    if status:
        print("✅ Status read successful:")
        print(f"  Busy: {status['busy']}")
        print(f"  Width: {status['width']}")
        print(f"  Force: {status['force']}")
        print(f"  Grip Detected: {status['grip_detected']}")
    else:
        print("❌ Status read failed")
    
    # Test command sending
    print("\n🎯 Testing command sending:")
    print("-" * 40)
    
    # Test moving to middle position with medium force
    test_width = 127  # Middle position (0-255)
    test_force = 127  # Medium force (0-255)
    
    success = client.send_gripper_command(test_width, test_force)
    if success:
        print(f"✅ Command sent successfully: width={test_width}, force={test_force}")
        
        # Wait and check status
        time.sleep(2)
        status = client.read_gripper_status()
        if status:
            print(f"📊 New status - Busy: {status['busy']}, Width: {status['width']}")
    else:
        print("❌ Command sending failed")
    
    client.disconnect()
    print("\n✅ Diagnostic completed")
    return 0

if __name__ == '__main__':
    sys.exit(main())