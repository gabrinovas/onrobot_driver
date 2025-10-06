#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from onrobot_driver.drivers.onrobot_gripper import OnRobotGripper

class OnRobotDriverNode(Node):
    def __init__(self):
        super().__init__('onrobot_driver_node')
        
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('gripper_type', '2FG7'),
                ('ip_address', '192.168.1.1'),
                ('port', 502),
                ('max_width', 0.085),
                ('min_width', 0.0),
                ('max_force', 100.0),
                ('update_rate', 100.0),
                ('simulation_mode', False),
            ]
        )
        
        # Initialize gripper controller
        self.gripper = OnRobotGripper(self)
        
        self.get_logger().info(f"OnRobot driver node started with gripper_type: {self.get_parameter('gripper_type').value}")
    
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