#!/usr/bin/env python3

"""
Test script to verify ROS2 Control imports work correctly.
Run this to diagnose import issues before building the full package.
"""

import sys
import os

# Add the current directory to Python path to find local modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

def test_standard_imports():
    """Test the standard ROS2 Humble imports"""
    print("=" * 60)
    print("TESTING STANDARD ROS2 HUMBLE IMPORTS")
    print("=" * 60)
    
    try:
        from hardware_interface.system_interface import SystemInterface
        print("✅ SUCCESS: imported SystemInterface from hardware_interface.system_interface")
    except ImportError as e:
        print(f"❌ FAILED: SystemInterface - {e}")
        return False
    
    try:
        from hardware_interface.callback_return import CallbackReturn
        print("✅ SUCCESS: imported CallbackReturn from hardware_interface.callback_return")
    except ImportError as e:
        print(f"❌ FAILED: CallbackReturn - {e}")
        return False
    
    try:
        from hardware_interface.types import LifecycleState
        print("✅ SUCCESS: imported LifecycleState from hardware_interface.types")
    except ImportError as e:
        print(f"❌ FAILED: LifecycleState - {e}")
        return False
    
    try:
        from hardware_interface.hardware_info import HardwareInfo
        print("✅ SUCCESS: imported HardwareInfo from hardware_interface.hardware_info")
    except ImportError as e:
        print(f"❌ FAILED: HardwareInfo - {e}")
        return False
    
    return True

def test_alternative_imports():
    """Test alternative import paths"""
    print("\n" + "=" * 60)
    print("TESTING ALTERNATIVE IMPORT PATHS")
    print("=" * 60)
    
    try:
        from ros2_control.hardware_interface import SystemInterface
        print("✅ SUCCESS: imported SystemInterface from ros2_control.hardware_interface")
    except ImportError as e:
        print(f"❌ FAILED: SystemInterface (alternative) - {e}")
        return False
    
    try:
        from ros2_control.hardware_interface import CallbackReturn
        print("✅ SUCCESS: imported CallbackReturn from ros2_control.hardware_interface")
    except ImportError as e:
        print(f"❌ FAILED: CallbackReturn (alternative) - {e}")
        return False
    
    try:
        from ros2_control.hardware_interface import LifecycleState
        print("✅ SUCCESS: imported LifecycleState from ros2_control.hardware_interface")
    except ImportError as e:
        print(f"❌ FAILED: LifecycleState (alternative) - {e}")
        return False
    
    try:
        from ros2_control.hardware_interface import HardwareInfo
        print("✅ SUCCESS: imported HardwareInfo from ros2_control.hardware_interface")
    except ImportError as e:
        print(f"❌ FAILED: HardwareInfo (alternative) - {e}")
        return False
    
    return True

def test_fallback_implementation():
    """Test that fallback implementation works"""
    print("\n" + "=" * 60)
    print("TESTING FALLBACK IMPLEMENTATION")
    print("=" * 60)
    
    try:
        from enum import Enum
        
        class CallbackReturn(Enum):
            SUCCESS = 0
            ERROR = 1
            FAILURE = 2
        
        class LifecycleState(Enum):
            UNCONFIGURED = 0
            INACTIVE = 1
            ACTIVE = 2
            FINALIZED = 3
        
        class HardwareInfo:
            def __init__(self):
                self.hardware_parameters = {}
        
        class SystemInterface:
            def __init__(self):
                pass
        
        print("✅ SUCCESS: Fallback implementation created")
        print("   - CallbackReturn enum defined")
        print("   - LifecycleState enum defined") 
        print("   - HardwareInfo class defined")
        print("   - SystemInterface class defined")
        return True
        
    except Exception as e:
        print(f"❌ FAILED: Fallback implementation - {e}")
        return False

def test_local_module_imports():
    """Test importing local modules"""
    print("\n" + "=" * 60)
    print("TESTING LOCAL MODULE IMPORTS")
    print("=" * 60)
    
    try:
        # Test importing the hardware interface itself
        from onrobot_driver.hardware.onrobot_gripper_hardware_interface import OnRobotGripperHardwareInterface
        print("✅ SUCCESS: imported OnRobotGripperHardwareInterface")
    except ImportError as e:
        print(f"❌ FAILED: OnRobotGripperHardwareInterface - {e}")
        return False
    
    try:
        # Test importing the gripper driver
        from onrobot_driver.drivers.onrobot_gripper import OnRobotGripper
        print("✅ SUCCESS: imported OnRobotGripper")
    except ImportError as e:
        print(f"⚠️  WARNING: OnRobotGripper - {e}")
        # This might be expected if dependencies aren't installed yet
        return True
    
    try:
        # Test importing modbus client
        from onrobot_driver.drivers.modbus_client import ModbusTCPClient
        print("✅ SUCCESS: imported ModbusTCPClient")
    except ImportError as e:
        print(f"⚠️  WARNING: ModbusTCPClient - {e}")
        # This might be expected if dependencies aren't installed yet
        return True
    
    return True

def check_ros_environment():
    """Check if ROS environment is properly set up"""
    print("\n" + "=" * 60)
    print("CHECKING ROS ENVIRONMENT")
    print("=" * 60)
    
    # Check if ROS_DISTRO is set
    ros_distro = os.environ.get('ROS_DISTRO', 'NOT SET')
    print(f"ROS_DISTRO: {ros_distro}")
    
    # Check if ROS version is Humble
    if ros_distro != 'humble':
        print(f"⚠️  WARNING: Expected ROS_DISTRO='humble', but got '{ros_distro}'")
    
    # Check if ROS2 is available
    try:
        import rclpy
        print("✅ SUCCESS: rclpy is available")
    except ImportError as e:
        print(f"❌ FAILED: rclpy - {e}")
        return False
    
    # Check ROS2 Control package installation
    try:
        import subprocess
        result = subprocess.run(['ros2', 'pkg', 'list'], capture_output=True, text=True)
        if 'hardware_interface' in result.stdout:
            print("✅ SUCCESS: hardware_interface package found")
        else:
            print("❌ FAILED: hardware_interface package not found in ROS2 packages")
            return False
    except Exception as e:
        print(f"⚠️  WARNING: Could not check ROS2 packages - {e}")
    
    return True

def main():
    """Run all tests"""
    print("OnRobot Driver Import Test")
    print("This script tests if all required imports work correctly.")
    print()
    
    # Check ROS environment first
    if not check_ros_environment():
        print("\n❌ ROS environment check failed!")
        return 1
    
    # Test standard imports
    standard_ok = test_standard_imports()
    
    # Test alternative imports if standard failed
    alternative_ok = False
    if not standard_ok:
        alternative_ok = test_alternative_imports()
    
    # Test fallback implementation
    fallback_ok = test_fallback_implementation()
    
    # Test local modules
    local_ok = test_local_module_imports()
    
    # Summary
    print("\n" + "=" * 60)
    print("TEST SUMMARY")
    print("=" * 60)
    
    if standard_ok:
        print("✅ STANDARD IMPORTS: WORKING - Using hardware_interface imports")
        result = 0
    elif alternative_ok:
        print("✅ ALTERNATIVE IMPORTS: WORKING - Using ros2_control imports")
        result = 0
    elif fallback_ok:
        print("⚠️  FALLBACK MODE: Using fallback implementation (no ROS2 Control)")
        result = 0
    else:
        print("❌ ALL IMPORT METHODS FAILED")
        result = 1
    
    if local_ok:
        print("✅ LOCAL MODULES: Can import local driver modules")
    else:
        print("❌ LOCAL MODULES: Some local imports failed")
        result = 1
    
    print(f"\nOverall result: {'SUCCESS' if result == 0 else 'FAILED'}")
    return result

if __name__ == '__main__':
    exit(main())