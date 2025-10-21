#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import threading
from collections import deque
import time

class MergedJointStatePublisher(Node):
    def __init__(self):
        super().__init__('merged_joint_state_publisher')
        
        self.arm_joint_state = None
        self.gripper_joint_state = None
        self.lock = threading.Lock()
        
        # Buffer for recent states to handle timing issues
        self.arm_states = deque(maxlen=10)
        self.gripper_states = deque(maxlen=10)
        
        # Subscribe to both sources
        self.arm_sub = self.create_subscription(
            JointState,
            'arm_joint_states',
            self.arm_callback,
            10
        )
        
        self.gripper_sub = self.create_subscription(
            JointState,
            'gripper_joint_states',
            self.gripper_callback, 
            10
        )
        
        # Publisher for merged states
        self.merged_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        # Timer for periodic merging (in case we miss some messages)
        self.merge_timer = self.create_timer(0.02, self.timer_callback)  # 50 Hz for better synchronization
        
        self.get_logger().info("Merged joint state publisher started")
        self.get_logger().info("Listening to: arm_joint_states, gripper_joint_states")
        self.get_logger().info("Publishing to: joint_states")
    
    def arm_callback(self, msg):
        with self.lock:
            self.arm_states.append(msg)
            self.arm_joint_state = msg  # Keep latest
            self.publish_merged()
    
    def gripper_callback(self, msg):
        with self.lock:
            self.gripper_states.append(msg)
            self.gripper_joint_state = msg  # Keep latest
            self.publish_merged()
    
    def timer_callback(self):
        """Fallback timer to ensure we publish even if messages are delayed"""
        with self.lock:
            self.publish_merged()
    
    def publish_merged(self):
        """Publish merged joint states if we have data from both sources"""
        merged = JointState()
        merged.header.stamp = self.get_clock().now().to_msg()
        merged.header.frame_id = ""  # No specific frame
        
        # Start with empty lists
        merged.name = []
        merged.position = []
        merged.velocity = []
        merged.effort = []
        
        # Add arm joints if available
        if self.arm_joint_state is not None:
            merged.name.extend(self.arm_joint_state.name)
            merged.position.extend(self.arm_joint_state.position)
            merged.velocity.extend(self.arm_joint_state.velocity)
            merged.effort.extend(self.arm_joint_state.effort)
        
        # Add gripper joints if available  
        if self.gripper_joint_state is not None:
            merged.name.extend(self.gripper_joint_state.name)
            merged.position.extend(self.gripper_joint_state.position)
            merged.velocity.extend(self.gripper_joint_state.velocity)
            merged.effort.extend(self.gripper_joint_state.effort)
        
        # Only publish if we have at least some joints
        if merged.name:
            self.merged_pub.publish(merged)
            
            # Debug logging - uncomment for troubleshooting
            if len(merged.name) >= 8:  # Should have 6 arm + 2 gripper joints
                self.get_logger().debug(f"✅ Published merged state with {len(merged.name)} joints", 
                                      throttle_duration_sec=5.0)
            else:
                self.get_logger().warn(f"⚠️ Incomplete joint state: {len(merged.name)} joints", 
                                     throttle_duration_sec=5.0)

def main():
    rclpy.init()
    node = MergedJointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down merged joint state publisher")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()