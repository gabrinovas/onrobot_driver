#!/usr/bin/env python3

import sys
import os

print("Final Import Test")
print("=" * 50)

# Test basic imports
try:
    import onrobot_driver
    print("✅ onrobot_driver package imported")
    
    # Test submodules
    from onrobot_driver.nodes import onrobot_driver_node
    print("✅ nodes imported")
    
    from onrobot_driver.drivers import onrobot_gripper, modbus_client
    print("✅ drivers imported")
    
    from onrobot_driver.hardware import onrobot_gripper_hardware_interface
    print("✅ hardware imported")
    
    # Test the actual hardware interface class
    from onrobot_driver.hardware.onrobot_gripper_hardware_interface import OnRobotGripperHardwareInterface
    print("✅ OnRobotGripperHardwareInterface imported")
    
    # Try to create an instance
    gripper = OnRobotGripperHardwareInterface()
    print("✅ OnRobotGripperHardwareInterface instance created")
    
    print("\n🎉 ALL IMPORTS SUCCESSFUL!")
    
except ImportError as e:
    print(f"❌ Import failed: {e}")
    print(f"Python path: {sys.path}")
    
    # Debug: show what's available
    print("\nAvailable in onrobot_driver:")
    try:
        import onrobot_driver
        print(dir(onrobot_driver))
    except:
        print("Could not import onrobot_driver")

if __name__ == '__main__':
    pass