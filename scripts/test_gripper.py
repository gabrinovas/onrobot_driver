#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
import time
import subprocess
import os

class GripperTester(Node):
    def __init__(self):
        super().__init__('gripper_tester')
        self.action_client = ActionClient(self, GripperCommand, '/gripper_action')
    
    def check_network_connectivity(self):
        """Run the network setup script to check hardware connectivity"""
        self.get_logger().info('Checking OnRobot hardware connectivity...')
        try:
            script_path = os.path.expanduser('~/setup_onrobot_network.sh')
            result = subprocess.run([script_path], capture_output=True, text=True)
            self.get_logger().info(f'Network check output:\n{result.stdout}')
            if result.stderr:
                self.get_logger().warn(f'Network check warnings:\n{result.stderr}')
            return "✅ Hardware mode enabled" in result.stdout
        except Exception as e:
            self.get_logger().warn(f'Network check failed: {e}')
            return False
        
    def test_gripper(self):
        # Check network first
        hardware_available = self.check_network_connectivity()
        
        if hardware_available:
            self.get_logger().info('Testing with REAL HARDWARE - be careful!')
            # Use smaller movements for real hardware
            positions = [0.06, 0.03, 0.06]  # Conservative movements
        else:
            self.get_logger().info('Testing in SIMULATION mode')
            # Use full range for simulation
            positions = [0.08, 0.04, 0.01, 0.06]  # Full range movements

        # Wait for action server
        self.get_logger().info('Waiting for gripper action server...')
        if not self.action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Gripper action server not found!')
            return False
        
        # Test sequence
        for i, pos in enumerate(positions):
            self.get_logger().info(f'Test {i+1}: Moving to position {pos}m...')
            
            goal_msg = GripperCommand.Goal()
            goal_msg.command.position = pos
            goal_msg.command.max_effort = 30.0 if hardware_available else 50.0
            
            future = self.action_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, future)
            
            # Wait longer for real hardware
            wait_time = 3.0 if hardware_available else 2.0
            time.sleep(wait_time)
        
        self.get_logger().info('All tests completed successfully!')
        return True

def main():
    rclpy.init()
    tester = GripperTester()
    success = tester.test_gripper()
    tester.destroy_node()
    rclpy.shutdown()
    
    exit(0 if success else 1)

if __name__ == '__main__':
    main()