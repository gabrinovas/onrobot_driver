#!/usr/bin/env python3
"""
Basic functionality test - run this first
"""
import rclpy
from rclpy.node import Node

def test_basic():
    print("🧪 BASIC FUNCTIONALITY TEST")
    print("=" * 50)
    
    # Test 1: ROS2 basic functionality
    try:
        rclpy.init()
        node = Node('test_basic')
        node.get_logger().info("✅ ROS2 basic functionality works")
        rclpy.shutdown()
        print("✅ ROS2 environment: PASS")
    except Exception as e:
        print(f"❌ ROS2 environment: FAIL - {e}")
        return False
    
    # Test 2: Import our driver
    try:
        from onrobot_driver.drivers.onrobot_gripper import OnRobotGripper
        print("✅ Driver imports: PASS")
    except Exception as e:
        print(f"❌ Driver imports: FAIL - {e}")
        return False
        
    print("🎉 All basic tests passed!")
    return True

if __name__ == '__main__':
    success = test_basic()
    exit(0 if success else 1)