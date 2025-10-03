#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
import time

class GripperTester(Node):
    def __init__(self):
        super().__init__('gripper_tester')
        self.action_client = ActionClient(self, GripperCommand, '/gripper_action')
        
    def test_gripper(self):
        # Wait for action server
        self.get_logger().info('Waiting for gripper action server...')
        self.action_client.wait_for_server()
        
        # Test open
        self.get_logger().info('Testing open gripper...')
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = 0.08  # Almost fully open
        goal_msg.command.max_effort = 50.0
        
        future = self.action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        
        time.sleep(2)
        
        # Test close
        self.get_logger().info('Testing close gripper...')
        goal_msg.command.position = 0.01  # Almost fully closed
        goal_msg.command.max_effort = 50.0
        
        future = self.action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        
        self.get_logger().info('Test completed!')

def main():
    rclpy.init()
    tester = GripperTester()
    tester.test_gripper()
    tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()