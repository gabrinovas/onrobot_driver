#!/usr/bin/env python3
"""
HARDWARE-ONLY TEST for OnRobot 2FG7 gripper
"""
import rclpy
from rclpy.node import Node
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
import time

class HardwareGripperTester(Node):
    def __init__(self):
        super().__init__('hardware_gripper_tester')
        self.action_client = ActionClient(self, GripperCommand, 'gripper_action')
    
    def wait_for_server(self, timeout=15.0):
        """Wait for gripper action server"""
        self.get_logger().info('Waiting for gripper action server...')
        return self.action_client.wait_for_server(timeout_sec=timeout)
    
    def test_basic_movements(self):
        """Test safe movements with real hardware"""
        test_sequence = [
            (0.070, 30.0, "Open to 70mm"),
            (0.050, 40.0, "Close to 50mm"),
            (0.060, 30.0, "Open to 60mm (ready)"),
        ]
        
        for position, force, description in test_sequence:
            self.get_logger().info(f'Testing: {description}')
            
            goal_msg = GripperCommand.Goal()
            goal_msg.command.position = position
            goal_msg.command.max_effort = force
            
            # Send goal
            future = self.action_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.result() is None:
                self.get_logger().error(f'Failed: {description} - No response')
                return False
                
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error(f'Failed: {description} - Rejected')
                return False
            
            self.get_logger().info(f'Accepted: {description}')
            
            # Wait for result
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=10.0)
            
            if result_future.result() is not None:
                result = result_future.result().result
                self.get_logger().info(f'Result: position={result.position:.3f}m, reached_goal={result.reached_goal}')
            
            time.sleep(2.0)  # Wait between movements
        
        return True

def main():
    rclpy.init()
    tester = HardwareGripperTester()
    
    try:
        if not tester.wait_for_server():
            tester.get_logger().error('Gripper action server not available!')
            exit(1)
        
        tester.get_logger().info('Starting hardware test sequence...')
        success = tester.test_basic_movements()
        
        if success:
            tester.get_logger().info('✅ Hardware test completed successfully!')
        else:
            tester.get_logger().error('❌ Hardware test failed!')
            exit(1)
            
    except KeyboardInterrupt:
        tester.get_logger().info('Test interrupted')
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()