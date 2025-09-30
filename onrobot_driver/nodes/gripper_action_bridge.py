#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time

class GripperActionBridge(Node):
    def __init__(self):
        super().__init__('gripper_action_bridge')
        
        # Action client for the gripper controller
        self.gripper_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/onrobot_2fg7_gripper_controller/follow_joint_trajectory'
        )
        
        self.get_logger().info("Gripper Action Bridge started")
        
        # Wait for action server
        self.get_logger().info("Waiting for gripper action server...")
        self.gripper_client.wait_for_server()
        self.get_logger().info("Gripper action server connected!")
    
    def move_gripper(self, position, duration=2.0):
        """Move gripper to specified position"""
        goal_msg = FollowJointTrajectory.Goal()
        
        # Create trajectory
        trajectory = JointTrajectory()
        trajectory.joint_names = ['left_finger_joint', 'right_finger_joint']
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = [position, position]  # Same position for both fingers
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        
        trajectory.points.append(point)
        goal_msg.trajectory = trajectory
        
        # Send goal
        self.get_logger().info(f"Sending gripper to position: {position}")
        future = self.gripper_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Gripper goal rejected')
            return
        
        self.get_logger().info('Gripper goal accepted')
        
        # Get result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Gripper movement completed with result: {result}')

def main():
    rclpy.init()
    node = GripperActionBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()