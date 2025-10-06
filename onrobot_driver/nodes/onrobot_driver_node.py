#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from onrobot_driver.drivers.onrobot_gripper import OnRobotGripper

class OnRobotDriverNode(Node):
    def __init__(self):
        super().__init__('onrobot_driver_node')
        
        # Initialize gripper controller - let it declare its own parameters
        self.gripper = OnRobotGripper(self)
        
        self.get_logger().info(f"OnRobot driver node started with gripper_type: {self.gripper.gripper_type}")
    
    def destroy_node(self):
        """Cleanup before shutdown"""
        self.get_logger().info("Shutting down OnRobot driver node")
        if hasattr(self, 'gripper'):
            self.gripper.disconnect()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = OnRobotDriverNode()
        
        # Use multi-threaded executor
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