#!/usr/bin/env python3

import rclpy
from std_srvs.srv import Trigger

def main():
    rclpy.init()
    node = rclpy.create_node('gripper_info_client')
    
    client = node.create_client(Trigger, 'gripper_debug_info')
    
    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error("Gripper debug service not available")
        return
    
    request = Trigger.Request()
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    
    if future.result() is not None:
        response = future.result()
        print("🎯 Gripper Debug Information:")
        print(response.message)
    else:
        print("❌ Failed to get gripper info")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()