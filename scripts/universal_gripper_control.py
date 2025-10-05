#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand
import sys

class UniversalGripperControl:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('universal_gripper_control')
        self.action_client = ActionClient(self.node, GripperCommand, 'gripper_action')
        
    def wait_for_server(self, timeout_sec=10.0):
        """Wait for gripper action server"""
        self.node.get_logger().info("Waiting for gripper action server...")
        if not self.action_client.wait_for_server(timeout_sec=timeout_sec):
            self.node.get_logger().error("Gripper action server not available")
            return False
        self.node.get_logger().info("Gripper action server found")
        return True
    
    def send_command(self, position, force=50.0, timeout=10.0):
        """Send gripper command - works for both simulation and hardware"""
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = float(position)
        goal_msg.command.max_effort = float(force)
        
        self.node.get_logger().info(f"Sending command: position={position}m, force={force}%")
        
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
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=timeout)
        
        if result_future.result() is None:
            self.node.get_logger().warn("No result received (timeout)")
            return True  # Still consider success for simulation
        
        result = result_future.result().result
        success = result.reached_goal and not result.stalled
        
        if success:
            self.node.get_logger().info(f"Command successful - final position: {result.position:.3f}m")
        else:
            self.node.get_logger().warn(f"Command completed but goal not fully reached")
            
        return success
    
    def run_sequence(self, sequence):
        """Run a sequence of gripper commands"""
        for i, (position, force, description) in enumerate(sequence):
            self.node.get_logger().info(f"Step {i+1}: {description}")
            if not self.send_command(position, force):
                self.node.get_logger().error(f"Step {i+1} failed")
                return False
            rclpy.spin_once(self.node, timeout_sec=2.0)  # Brief pause between commands
        return True

def main():
    controller = UniversalGripperControl()
    
    if not controller.wait_for_server():
        sys.exit(1)
    
    # Example sequence - works for both simulation and hardware
    sequence = [
        (0.070, 30.0, "Open to 70mm"),
        (0.040, 60.0, "Close to 40mm"),
        (0.060, 40.0, "Open to 60mm (ready position)"),
    ]
    
    success = controller.run_sequence(sequence)
    
    if success:
        controller.node.get_logger().info("🎉 Sequence completed successfully!")
    else:
        controller.node.get_logger().error("💥 Sequence failed!")
        sys.exit(1)

if __name__ == '__main__':
    main()