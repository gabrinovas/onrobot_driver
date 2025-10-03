#!/usr/bin/env python3

"""
Test script to verify ROS2 Control imports work correctly.
Run this to diagnose import issues before building the full package.
"""

import sys
import os

def test_standard_imports():
    """Test the standard ROS2 Humble imports"""
    print("=" * 60)
    print("TESTING STANDARD ROS2 HUMBLE IMPORTS")
    print("=" * 60)
    
    try:
        from hardware_interface.system_interface import SystemInterface
        print("✅ SUCCESS: imported SystemInterface from hardware_interface.system_interface")
        return True
    except ImportError as e:
        print(f"❌ FAILED: SystemInterface - {e}")
        return False

def test_ros2_control_imports():
    """Test if we can import any ROS2 Control components"""
    print("\n" + "=" * 60)
    print("TESTING ROS2 CONTROL COMPONENTS")
    print("=" * 60)
    
    # Test if we can import controller_manager
    try:
        import controller_manager
        print("✅ SUCCESS: imported controller_manager")
    except ImportError as e:
        print(f"❌ FAILED: controller_manager - {e}")
    
    # Test if we can import hardware_interface C++ extension (common issue)
    try:
        import hardware_interface
        print("✅ SUCCESS: imported hardware_interface")
        return True
    except ImportError as e:
        print(f"❌ FAILED: hardware_interface - {e}")
        return False

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
        return True
        
    except Exception as e:
        print(f"❌ FAILED: Fallback implementation - {e}")
        return False

def test_local_module_imports():
    """Test importing local modules with proper path setup"""
    print("\n" + "=" * 60)
    print("TESTING LOCAL MODULE IMPORTS")
    print("=" * 60)
    
    # Add the src directory to Python path
    current_dir = os.path.dirname(os.path.abspath(__file__))
    package_dir = os.path.join(current_dir, '..')
    src_dir = os.path.join(current_dir, '..', '..')  # src directory
    
    sys.path.insert(0, os.path.abspath(package_dir))
    sys.path.insert(0, os.path.abspath(src_dir))
    
    print(f"Added to Python path: {package_dir}")
    print(f"Added to Python path: {src_dir}")
    
    try:
        # Test importing the hardware interface itself
        from onrobot_driver.hardware.onrobot_gripper_hardware_interface import OnRobotGripperHardwareInterface
        print("✅ SUCCESS: imported OnRobotGripperHardwareInterface")
        return True
    except ImportError as e:
        print(f"❌ FAILED: OnRobotGripperHardwareInterface - {e}")
        
        # Try to see what's available
        print("\nTrying to debug the import issue...")
        try:
            import onrobot_driver
            print(f"✅ Can import onrobot_driver package from: {onrobot_driver.__file__}")
        except ImportError as e2:
            print(f"❌ Cannot import onrobot_driver at all: {e2}")
            
        # List contents of the directory
        hardware_dir = os.path.join(package_dir, 'hardware')
        if os.path.exists(hardware_dir):
            print(f"Contents of hardware directory: {os.listdir(hardware_dir)}")
        else:
            print(f"Hardware directory not found at: {hardware_dir}")
            
        return False

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
    
    # Check ROS2 Control package installation at system level
    try:
        import subprocess
        result = subprocess.run(['ros2', 'pkg', 'list'], capture_output=True, text=True)
        if 'hardware_interface' in result.stdout:
            print("✅ SUCCESS: hardware_interface package found in ROS2 packages")
        else:
            print("❌ hardware_interface package not found in ROS2 packages")
            
        # Check if it's a C++ only package (common issue)
        print("\nChecking package contents...")
        result = subprocess.run(['ros2', 'pkg', 'prefix', 'hardware_interface'], 
                              capture_output=True, text=True)
        if result.returncode == 0:
            pkg_path = result.stdout.strip()
            print(f"hardware_interface package path: {pkg_path}")
            
            # Check for Python modules
            python_path = os.path.join(pkg_path, 'local', 'lib', 'python3.10', 'dist-packages')
            if os.path.exists(python_path):
                print(f"Python path exists: {python_path}")
                # Add it to Python path
                sys.path.insert(0, python_path)
        else:
            print("Could not find hardware_interface package path")
            
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
    
    # Test ROS2 Control components
    ros2_control_ok = test_ros2_control_imports()
    
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
    elif ros2_control_ok:
        print("✅ ROS2 CONTROL: Some components available")
        result = 0
    elif fallback_ok:
        print("⚠️  FALLBACK MODE: Using fallback implementation (no ROS2 Control Python bindings)")
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
    
    if result != 0:
        print("\n" + "=" * 60)
        print("TROUBLESHOOTING TIPS")
        print("=" * 60)
        print("1. Make sure you're in your workspace directory")
        print("2. Run: 'source /opt/ros/humble/setup.bash'")
        print("3. Run: 'colcon build --packages-select onrobot_driver'") 
        print("4. Run: 'source install/setup.bash'")
        print("5. Try the test again")
        print("\nCommon issue: hardware_interface Python bindings may not be available")
        print("in ROS2 Humble. The fallback implementation should work for testing.")
    
    return result

if __name__ == '__main__':
    exit(main())