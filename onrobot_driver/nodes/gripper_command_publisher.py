#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import time

class GripperCommandPublisher(Node):
    def __init__(self):
        super().__init__('gripper_command_publisher')
        
        # Publishers for joint commands
        self.left_finger_pub = self.create_publisher(Float64, '/left_finger_joint/commands', 10)
        self.right_finger_pub = self.create_publisher(Float64, '/right_finger_joint/commands', 10)
        
        # Set initial position (open gripper)
        self.set_gripper_position(0.08)  # Open position
        
        self.get_logger().info("Gripper Command Publisher started - Gripper set to open position")
    
    def set_gripper_position(self, position):
        """Set both finger joints to the same position"""
        left_cmd = Float64()
        left_cmd.data = position
        self.left_finger_pub.publish(left_cmd)
        
        right_cmd = Float64()
        right_cmd.data = position
        self.right_finger_pub.publish(right_cmd)
        
        self.get_logger().info(f"Set gripper position to: {position}")

def main():
    rclpy.init()
    node = GripperCommandPublisher()
    
    try:
        # Keep the node alive to maintain the position
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()