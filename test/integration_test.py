#!/usr/bin/env python3
"""
Fixed integration test that starts everything properly
"""
import subprocess
import time
import threading
import rclpy
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand

class IntegrationTest:
    def __init__(self):
        self.mock_process = None
        self.driver_process = None
        
    def start_mock_server(self):
        """Start mock server in background"""
        print("🚀 Starting mock gripper server...")
        self.mock_process = subprocess.Popen(
            ['python3', 'test/mock_gripper_server.py'],
            cwd='src/onrobot_driver'
        )
        time.sleep(2)
        
    def start_driver_node(self):
        """Start driver node in background"""
        print("🚀 Starting driver node...")
        self.driver_process = subprocess.Popen([
            'install/onrobot_driver/lib/onrobot_driver/onrobot_driver_node',
            '--ros-args',
            '-p', 'ip_address:=127.0.0.1',
            '-p', 'port:=1502', 
            '-p', 'update_rate:=10.0'
        ])
        time.sleep(3)
        
    def test_gripper_actions(self):
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
    
    def cleanup(self):
        """Cleanup processes"""
        if self.driver_process:
            self.driver_process.terminate()
            self.driver_process.wait()
        if self.mock_process:
            self.mock_process.terminate()
            self.mock_process.wait()

def main():
    test = IntegrationTest()
    
    try:
        # Start services
        test.start_mock_server()
        test.start_driver_node()
        
        # Run test
        success = test.test_gripper_actions()
        print(f"\n🎯 INTEGRATION TEST: {'PASS' if success else 'FAIL'}")
        
    except Exception as e:
        print(f"❌ Test setup failed: {e}")
        success = False
    finally:
        test.cleanup()
    
    exit(0 if success else 1)

if __name__ == '__main__':
    main()