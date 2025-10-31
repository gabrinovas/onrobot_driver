#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand
import sys
import time
import argparse

class GripperController:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('gripper_controller')
        
        # Action client for the gripper action server
        self.action_client = ActionClient(
            self.node, 
            GripperCommand, 
            'gripper_action'
        )
        
    def wait_for_server(self, timeout_sec=10.0):
        """Wait for gripper action server"""
        self.node.get_logger().info("Waiting for gripper action server...")
        if not self.action_client.wait_for_server(timeout_sec=timeout_sec):
            self.node.get_logger().error("Gripper action server not available")
            return False
        self.node.get_logger().info("Gripper action server found")
        return True
    
    def control_gripper(self, position, force=50.0, timeout=30.0):
        """Control gripper position with enhanced feedback"""
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = float(position)
        goal_msg.command.max_effort = float(force)
        
        self.node.get_logger().info(f"Moving gripper to: {position:.3f}m, force: {force:.1f}N")
        
        # Send goal
        future = self.action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        )
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        
        if future.result() is None:
            self.node.get_logger().error("Command failed - no response")
            return False
            
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error("Command rejected by server")
            return False
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if result_future.done():
                break
        
        if not result_future.done():
            self.node.get_logger().warn("Command timed out")
            return False
        
        result = result_future.result().result
        success = result.reached_goal and not result.stalled
        
        if success:
            self.node.get_logger().info(f"✅ Command successful - final position: {result.position:.3f}m, force: {result.effort:.1f}N")
        else:
            self.node.get_logger().warn(f"⚠️ Command completed but goal not fully reached")
            self.node.get_logger().warn(f"   Final position: {result.position:.3f}m, stalled: {result.stalled}")
            
        return success
    
    def feedback_callback(self, feedback_msg):
        """Handle action feedback"""
        feedback = feedback_msg.feedback
        self.node.get_logger().info(
            f"📊 Progress - Position: {feedback.position:.3f}m, "
            f"Force: {feedback.effort:.1f}N"
        )
    
    def get_current_status(self):
        """Get current gripper status by checking available topics"""
        # This is a simple implementation - in practice you'd subscribe to status topics
        self.node.get_logger().info("Use 'ros2 topic echo' commands to monitor gripper status:")
        self.node.get_logger().info("  ros2 topic echo /joint_states")
        self.node.get_logger().info("  ros2 topic echo /gripper_position")
        self.node.get_logger().info("  ros2 topic echo /gripper_status")

def main():
    parser = argparse.ArgumentParser(description='Control OnRobot Gripper')
    parser.add_argument('position', type=float, nargs='?', 
                       help='Target position in meters (e.g., 0.04 for 40mm)')
    parser.add_argument('--force', '-f', type=float, default=50.0,
                       help='Gripping force in Newtons (default: 50.0)')
    parser.add_argument('--timeout', '-t', type=float, default=30.0,
                       help='Command timeout in seconds (default: 30.0)')
    parser.add_argument('--status', '-s', action='store_true',
                       help='Show current gripper status')
    
    args = parser.parse_args()
    
    controller = GripperController()
    
    if not controller.wait_for_server():
        sys.exit(1)
    
    if args.status:
        controller.get_current_status()
        controller.node.destroy_node()
        rclpy.shutdown()
        return
    
    if args.position is None:
        parser.print_help()
        controller.node.destroy_node()
        rclpy.shutdown()
        return
    
    success = controller.control_gripper(args.position, args.force, args.timeout)
    
    if success:
        print("✅ Gripper command executed successfully")
    else:
        print("❌ Gripper command failed")
        sys.exit(1)
    
    controller.node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()