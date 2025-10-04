#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class SimpleTestNode(Node):
    def __init__(self):
        super().__init__('simple_test_node')
        self.get_logger().info('Simple test node started!')
        
        # Create a timer to keep the node alive
        self.timer = self.create_timer(2.0, self.timer_callback)
    
    def timer_callback(self):
        self.get_logger().info('Test node is running...')

def main():
    rclpy.init()
    node = SimpleTestNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()