#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand
import sys
import time

class GripperController:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('gripper_controller')
        
        # Action client for the gripper action server
        self.action_client = ActionClient(
            self.node, 
            GripperCommand, 
            'gripper_action'  # Use the original action name
        )
        
    def wait_for_server(self, timeout_sec=10.0):
        """Wait for gripper action server"""
        self.node.get_logger().info("Waiting for gripper action server...")
        if not self.action_client.wait_for_server(timeout_sec=timeout_sec):
            self.node.get_logger().error("Gripper action server not available")
            return False
        self.node.get_logger().info("Gripper action server found")
        return True
    
    def control_gripper(self, position, force=50.0):
        """Control gripper position"""
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = float(position)
        goal_msg.command.max_effort = float(force)
        
        self.node.get_logger().info(f"Moving gripper to: {position}m, force: {force}%")
        
        # Send goal
        future = self.action_client.send_goal_async(goal_msg)
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
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=10.0)
        
        if result_future.result() is None:
            self.node.get_logger().warn("No result received (timeout)")
            return True  # Still consider success
        
        result = result_future.result().result
        success = result.reached_goal and not result.stalled
        
        if success:
            self.node.get_logger().info(f"Command successful - final position: {result.position:.3f}m")
        else:
            self.node.get_logger().warn(f"Command completed but goal not fully reached")
            
        return success

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 control_gripper.py <position_in_meters> [force_percentage]")
        print("Example: python3 control_gripper.py 0.04 60.0")
        sys.exit(1)
    
    position = float(sys.argv[1])
    force = float(sys.argv[2]) if len(sys.argv) > 2 else 50.0
    
    controller = GripperController()
    
    if not controller.wait_for_server():
        sys.exit(1)
    
    success = controller.control_gripper(position, force)
    
    if success:
        print("✅ Gripper command executed successfully")
    else:
        print("❌ Gripper command failed")
        sys.exit(1)
    
    controller.node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()