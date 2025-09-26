#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import threading
import time

from onrobot_driver.drivers.onrobot_gripper import OnRobotGripper

class OnRobotDriverNode(Node):
    def __init__(self):
        super().__init__('onrobot_driver_node')
        
        # Get gripper type parameter
        self.declare_parameter('gripper_type', '2FG7')
        gripper_type = self.get_parameter('gripper_type').value
        
        # Initialize gripper controller
        self.gripper = OnRobotGripper(self, gripper_type)
        
        # Create timer for status updates
        update_rate = self.gripper.update_rate
        self.timer = self.create_timer(1.0/update_rate, self.timer_callback)
        
        self.get_logger().info(f"OnRobot {gripper_type} driver node started")
    
    def timer_callback(self):
        """Timer callback for periodic status updates"""
        try:
            self.gripper.publish_status()
        except Exception as e:
            self.get_logger().error(f"Error in timer callback: {e}")
    
    def destroy_node(self):
        """Cleanup before shutdown"""
        self.get_logger().info("Shutting down OnRobot driver node")
        self.gripper.disconnect()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = OnRobotDriverNode()
        
        # Use multi-threaded executor for action server
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        
        try:
            executor.spin()
        finally:
            executor.shutdown()
            node.destroy_node()
            
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in main: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()