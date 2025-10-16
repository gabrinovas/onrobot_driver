#!/usr/bin/env python3
"""
Quick test for basic gripper functionality
"""
import rclpy
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand
import sys

def quick_test():
    rclpy.init()
    
    from rclpy.node import Node
    node = Node('quick_test')
    
    action_client = ActionClient(node, GripperCommand, 'gripper_action')
    
    print("Waiting for gripper action server...")
    if not action_client.wait_for_server(timeout_sec=10.0):
        print("❌ Action server not available")
        return False
    
    print("✅ Action server found")
    
    # Try a simple command
    goal_msg = GripperCommand.Goal()
    goal_msg.command.position = 0.06  # Middle position
    goal_msg.command.max_effort = 30.0
    
    print("Sending test command...")
    future = action_client.send_goal_async(goal_msg)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    
    if future.result() is None:
        print("❌ No response to command")
        return False
        
    goal_handle = future.result()
    if goal_handle.accepted:
        print("✅ Command accepted by server")
        return True
    else:
        print("❌ Command rejected")
        return False

if __name__ == '__main__':
    success = quick_test()
    print("🎉 Quick test passed!" if success else "💥 Quick test failed!")
    exit(0 if success else 1)