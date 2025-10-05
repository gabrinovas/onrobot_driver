#!/usr/bin/env python3
"""
Integration test with mock server
"""
import subprocess
import time
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

def start_driver_node():
    """Start driver node in background"""
    print("🚀 Starting driver node...")
    process = subprocess.Popen([
        'ros2', 'run', 'onrobot_driver', 'onrobot_driver_node',
        '--ros-args',
        '-p', 'ip_address:=127.0.0.1',
        '-p', 'port:=1502',
        '-p', 'gripper_type:=2FG7',
        '-p', 'update_rate:=10.0'
    ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    time.sleep(3)  # Give driver time to start
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
        
        # Wait for server with longer timeout
        print("Waiting for gripper action server...")
        if not action_client.wait_for_server(timeout_sec=15.0):
            print("❌ Action server not available")
            print("Make sure the driver node is running")
            return False
            
        print("✅ Action server: AVAILABLE")
        
        # Test basic command
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = 0.05
        goal_msg.command.max_effort = 40.0
        
        print("Sending test command...")
        future = action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
        
        if future.result() is None:
            print("❌ Command failed - no response")
            return False
            
        goal_handle = future.result()
        if goal_handle.accepted:
            print("✅ Action command: ACCEPTED")
            return True
        else:
            print("❌ Action command: REJECTED")
            return False
        
    except Exception as e:
        print(f"❌ Integration test failed: {e}")
        return False
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    # Start mock server
    server_process = start_mock_server()
    driver_process = None
    
    try:
        # Start driver node
        driver_process = start_driver_node()
        
        # Run integration test
        success = test_gripper_actions()
        print(f"\n🎯 INTEGRATION TEST: {'PASS' if success else 'FAIL'}")
        
    except Exception as e:
        print(f"❌ Test setup failed: {e}")
        success = False
    finally:
        # Cleanup
        if driver_process:
            driver_process.terminate()
            driver_process.wait()
        server_process.terminate()
        server_process.wait()
    
    exit(0 if success else 1)