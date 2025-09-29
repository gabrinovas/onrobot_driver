#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from control_msgs.action import GripperCommand
from std_msgs.msg import Float64
import threading
import time

class GripperActionServer(Node):
    def __init__(self):
        super().__init__('gripper_action_server')
        
        # Action server for gripper commands
        self.action_server = ActionServer(
            self,
            GripperCommand,
            'onrobot_2fg7_gripper_controller/gripper_action',
            self.execute_callback
        )
        
        # Publisher for joint commands
        self.left_finger_pub = self.create_publisher(Float64, '/left_finger_joint/commands', 10)
        self.right_finger_pub = self.create_publisher(Float64, '/right_finger_joint/commands', 10)
        
        self.get_logger().info("Gripper Action Server started")
    
    async def execute_callback(self, goal_handle):
        goal = goal_handle.request.command
        position = goal.position
        max_effort = goal.max_effort
        
        self.get_logger().info(f"Executing gripper command: position={position}, effort={max_effort}")
        
        # Send command to both finger joints
        left_cmd = Float64()
        left_cmd.data = position
        self.left_finger_pub.publish(left_cmd)
        
        right_cmd = Float64()
        right_cmd.data = position  # Same position for both fingers
        self.right_finger_pub.publish(right_cmd)
        
        # Simulate execution
        time.sleep(1.0)
        
        # Return result
        result = GripperCommand.Result()
        result.position = position
        result.effort = max_effort
        result.reached_goal = True
        result.stalled = False
        
        goal_handle.succeed(result)
        return result

def main():
    rclpy.init()
    node = GripperActionServer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()