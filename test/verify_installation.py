#!/usr/bin/env python3
"""
Verify that the driver can be installed and run
"""
import os
import sys
import subprocess

def verify_installation():
    print("🔍 VERIFYING ONROBOT DRIVER INSTALLATION")
    print("=" * 50)
    
    # Check 1: Python imports
    print("1. Testing Python imports...")
    try:
        from onrobot_driver.nodes.onrobot_driver_node import main as node_main
        print("✅ onrobot_driver_node import: SUCCESS")
    except ImportError as e:
        print(f"❌ onrobot_driver_node import: FAILED - {e}")
        return False
    
    try:
        from onrobot_driver.drivers.onrobot_gripper import OnRobotGripper
        print("✅ OnRobotGripper import: SUCCESS")
    except ImportError as e:
        print(f"❌ OnRobotGripper import: FAILED - {e}")
        return False
    
    # Check 2: ROS2 package registration
    print("\n2. Testing ROS2 package registration...")
    try:
        result = subprocess.run(['ros2', 'pkg', 'list'], capture_output=True, text=True)
        if 'onrobot_driver' in result.stdout:
            print("✅ ROS2 package registration: SUCCESS")
        else:
            print("❌ ROS2 package registration: FAILED - package not found")
            return False
    except Exception as e:
        print(f"❌ ROS2 package check failed: {e}")
        return False
    
    # Check 3: Executable availability
    print("\n3. Testing executable availability...")
    try:
        result = subprocess.run(['ros2', 'pkg', 'executables', 'onrobot_driver'], 
                              capture_output=True, text=True)
        if 'onrobot_driver_node' in result.stdout:
            print("✅ Executable registration: SUCCESS")
            print(f"Executables found:\n{result.stdout}")
        else:
            print("❌ Executable registration: FAILED")
            print("Available executables:")
            print(result.stdout)
            return False
    except Exception as e:
        print(f"❌ Executable check failed: {e}")
        return False
    
    print("\n🎉 INSTALLATION VERIFICATION: SUCCESS")
    print("The driver is properly installed and should be runnable with:")
    print("  ros2 run onrobot_driver onrobot_driver_node")
    return True

if __name__ == '__main__':
    success = verify_installation()
    sys.exit(0 if success else 1)