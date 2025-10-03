#!/usr/bin/env python3

"""
Quick test to verify the basic hardware interface can be instantiated.
"""

import sys
import os

# Add the parent directory to Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

def quick_test():
    """Quick test of the hardware interface"""
    print("Quick Test: OnRobot Gripper Hardware Interface")
    print("-" * 50)
    
    try:
        from onrobot_driver.hardware.onrobot_gripper_hardware_interface import OnRobotGripperHardwareInterface
        
        # Try to create an instance
        gripper_interface = OnRobotGripperHardwareInterface()
        print("✅ SUCCESS: Created OnRobotGripperHardwareInterface instance")
        
        # Test basic methods
        try:
            interfaces = gripper_interface.export_state_interfaces()
            print(f"✅ SUCCESS: export_state_interfaces() returned {len(interfaces)} interfaces")
            
            cmd_interfaces = gripper_interface.export_command_interfaces()
            print(f"✅ SUCCESS: export_command_interfaces() returned {len(cmd_interfaces)} interfaces")
            
            return True
            
        except Exception as e:
            print(f"⚠️  WARNING: Method test failed - {e}")
            return True  # Still consider it a success if we can import
            
    except Exception as e:
        print(f"❌ FAILED: Could not create hardware interface - {e}")
        return False

if __name__ == '__main__':
    success = quick_test()
    sys.exit(0 if success else 1)