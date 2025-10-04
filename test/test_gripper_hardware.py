#!/usr/bin/env python3
"""
HARDWARE-ONLY TEST for OnRobot gripper
This test will FAIL if no real hardware is connected
Use only for production testing with actual gripper
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
    
    def test_hardware_connection(self):
        """Test that gripper hardware is available and responsive"""
        self.get_logger().info('=== OnRobot Gripper HARDWARE TEST ===')
        self.get_logger().info('This test requires REAL HARDWARE connected to 192.168.1.1')
        
        # Wait for action server (indicates driver is running and connected to hardware)
        self.get_logger().info('Waiting for gripper action server...')
        if not self.action_client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error('❌ Gripper action server not available!')
            self.get_logger().error('This usually means:')
            self.get_logger().error('1. Driver cannot connect to hardware')
            self.get_logger().error('2. Compute Box is not reachable at 192.168.1.1')
            self.get_logger().error('3. Gripper is not powered on or connected')
            return False
        
        self.get_logger().info('✅ Gripper action server available - hardware connection established')
        return True
    
    def run_production_test_sequence(self):
        """Run safe test sequence for real hardware"""
        self.get_logger().info('Running PRODUCTION test sequence...')
        
        # Safe test positions for 2FG7 gripper (35mm to 75mm range)
        test_sequence = [
            (0.070, 30.0, "Open to 70mm (safe position)"),
            (0.050, 40.0, "Close to 50mm (medium grip)"),
            (0.060, 30.0, "Open to 60mm (ready position)"),
        ]
        
        all_success = True
        
        for position, force, description in test_sequence:
            self.get_logger().info(f'🧪 Testing: {description}')
            
            success = self.send_gripper_command(position, force)
            
            if success:
                self.get_logger().info(f'✅ {description} - SUCCESS')
            else:
                self.get_logger().error(f'❌ {description} - FAILED')
                all_success = False
            
            # Wait for gripper to complete movement
            time.sleep(3.0)
        
        return all_success
    
    def send_gripper_command(self, position: float, force: float) -> bool:
        """Send command to real hardware and verify result"""
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = force
        
        # Send goal with timeout
        future = self.action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is None:
            self.get_logger().error('Goal rejected - timeout')
            return False
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by server')
            return False
        
        # Wait for result with timeout
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=10.0)
        
        if result_future.result() is None:
            self.get_logger().error('No result received - hardware may be stuck')
            return False
        
        result = result_future.result().result
        self.get_logger().info(f'Hardware result: position={result.position:.3f}m, '
                              f'reached_goal={result.reached_goal}, stalled={result.stalled}')
        
        return result.reached_goal and not result.stalled

def main():
    rclpy.init()
    
    tester = HardwareGripperTester()
    
    try:
        # Test hardware connection
        if not tester.test_hardware_connection():
            tester.get_logger().error('💥 HARDWARE TEST FAILED - No gripper connected')
            exit(1)
        
        # Run production test sequence
        test_success = tester.run_production_test_sequence()
        
        # Final results
        tester.get_logger().info('')
        tester.get_logger().info('=== PRODUCTION TEST SUMMARY ===')
        
        if test_success:
            tester.get_logger().info('🎉 PRODUCTION READY - OnRobot gripper hardware working correctly!')
            exit_code = 0
        else:
            tester.get_logger().error('💥 PRODUCTION TEST FAILED - Check hardware connection and configuration')
            exit_code = 1
            
    except KeyboardInterrupt:
        tester.get_logger().info('Test interrupted by user')
        exit_code = 0
    except Exception as e:
        tester.get_logger().error(f'Hardware test failed with exception: {e}')
        exit_code = 1
    finally:
        tester.destroy_node()
        rclpy.shutdown()
    
    exit(exit_code)

if __name__ == '__main__':
    main()