#!/usr/bin/env python3
"""
Integration test with mock server
"""
import subprocess
import time
import threading
import rclpy
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand

def start_mock_server():
    """Start mock server in background"""
    print("🚀 Starting mock gripper server...")
    process = subprocess.Popen(
        ['python3', 'test/mock_gripper_server.py'],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )
    time.sleep(2)  # Give server time to start
    return process

def test_gripper_actions():
    """Test gripper action server"""
    print("🧪 Testing gripper action server...")
    
    rclpy.init()
    
    try:
        from rclpy.node import Node
        node = Node('test_integration')
        
        # Create action client
        action_client = ActionClient(node, GripperCommand, '/gripper_action')
        
        # Wait for server
        if not action_client.wait_for_server(timeout_sec=10.0):
            print("❌ Action server not available")
            return False
            
        print("✅ Action server: AVAILABLE")
        
        # Test basic command
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = 0.05
        goal_msg.command.max_effort = 40.0
        
        future = action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(node, future)
        
        print("✅ Action command: SENT SUCCESSFULLY")
        return True
        
    except Exception as e:
        print(f"❌ Integration test failed: {e}")
        return False
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    # Start mock server
    server_process = start_mock_server()
    
    try:
        # Run integration test
        success = test_gripper_actions()
        print(f"\n🎯 INTEGRATION TEST: {'PASS' if success else 'FAIL'}")
    finally:
        # Cleanup
        server_process.terminate()
        server_process.wait()