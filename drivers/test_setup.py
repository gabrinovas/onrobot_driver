#!/usr/bin/env python3
import os
import sys

def check_files():
    required_files = [
        'hardware/onrobot_gripper_plugin.xml',
        'CMakeLists.txt',
        'package.xml',
        'drivers/modbus_client.py',
        'nodes/onrobot_driver_node.py'
    ]
    
    print("🔍 Checking required files...")
    for file_path in required_files:
        if os.path.exists(file_path):
            print(f"✅ {file_path}")
        else:
            print(f"❌ {file_path} - MISSING!")
            return False
    return True

def check_imports():
    print("\n🔍 Checking Python imports...")
    try:
        import rclpy
        print("✅ rclpy")
    except ImportError as e:
        print(f"❌ rclpy: {e}")
        return False
        
    try:
        from control_msgs.action import GripperCommand
        print("✅ control_msgs")
    except ImportError as e:
        print(f"❌ control_msgs: {e}")
        return False
        
    try:
        import pymodbus
        print("✅ pymodbus")
    except ImportError as e:
        print(f"❌ pymodbus: {e}")
        return False
        
    return True

if __name__ == "__main__":
    print("🚀 OnRobot Driver Setup Check")
    print("=" * 40)
    
    files_ok = check_files()
    imports_ok = check_imports()
    
    print("\n" + "=" * 40)
    if files_ok and imports_ok:
        print("🎉 All checks passed! Ready to compile.")
    else:
        print("💥 Some checks failed. Please fix before compiling.")
        sys.exit(1)