#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from control_msgs.action import GripperCommand
from moveit_msgs.msg import CollisionObject, PlanningScene, AttachedCollisionObject
from moveit_msgs.srv import GetPlanningScene, ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Header, Float32, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
import math
import threading
from typing import Dict, Any
import asyncio


class MoveItGripperBridge(Node):
    """
    Enhanced MoveIt Gripper Bridge for OnRobot 2FG7 with asymmetric finger configuration.
    Handles collision object updates and gripper control integration with MoveIt.
    """

    def __init__(self):
        super().__init__('moveit_gripper_bridge')
        
        # Use reentrant callback group for parallel action handling
        self.callback_group = ReentrantCallbackGroup()
        
        # Parameters for asymmetric gripper configuration
        self.declare_parameters(
            namespace='',
            parameters=[
                ('update_rate', 20.0),
                ('publish_markers', True),
                ('left_finger_initial_x', 0.032239),
                ('right_finger_initial_x', -0.054361),
                ('finger_y_position', 0.029494),
                ('finger_z_position', -0.12005),
                ('finger_thickness', 0.005),
                ('finger_length', 0.03),
                ('finger_height', 0.01),
                ('base_link_frame', '2fg7_base_link'),
            ]
        )
        
        # Get parameters
        self.update_rate = self.get_parameter('update_rate').value
        self.publish_markers = self.get_parameter('publish_markers').value
        self.left_finger_initial_x = self.get_parameter('left_finger_initial_x').value
        self.right_finger_initial_x = self.get_parameter('right_finger_initial_x').value
        self.finger_y_position = self.get_parameter('finger_y_position').value
        self.finger_z_position = self.get_parameter('finger_z_position').value
        self.finger_thickness = self.get_parameter('finger_thickness').value
        self.finger_length = self.get_parameter('finger_length').value
        self.finger_height = self.get_parameter('finger_height').value
        self.base_link_frame = self.get_parameter('base_link_frame').value
        
        # Gripper state
        self.current_width = 0.07  # Start fully open (70mm)
        self.target_width = 0.07
        self.is_moving = False
        self.lock = threading.Lock()
        
        # Action client for OnRobot gripper
        self.gripper_action_client = ActionClient(
            self, 
            GripperCommand, 
            'gripper_action',
            callback_group=self.callback_group
        )
        
        # Publishers
        self.collision_object_pub = self.create_publisher(
            CollisionObject, 
            'collision_object', 
            10
        )
        
        self.planning_scene_pub = self.create_publisher(
            PlanningScene,
            'planning_scene',
            10
        )
        
        self.marker_pub = self.create_publisher(
            MarkerArray,
            'gripper_visualization_markers',
            10
        )
        
        # Subscribers
        self.gripper_sub = self.create_subscription(
            Float32,
            'gripper_position',
            self.gripper_position_callback,
            10
        )
        
        # Service clients for planning scene
        self.planning_scene_client = self.create_client(
            GetPlanningScene, 
            'get_planning_scene'
        )
        
        self.apply_planning_scene_client = self.create_client(
            ApplyPlanningScene,
            'apply_planning_scene'
        )
        
        # Timers
        self.update_timer = self.create_timer(
            1.0 / self.update_rate, 
            self.update_collision_objects
        )
        
        # Collision object IDs
        self.collision_ids = {
            'left_finger': 'left_finger_collision',
            'right_finger': 'right_finger_collision', 
            'gripper_opening': 'gripper_opening_space'
        }
        
        # Initialize collision objects
        self.initialize_collision_objects()
        
        self.get_logger().info("🤖 MoveIt Gripper Bridge initialized")
        self.get_logger().info(f"📏 Gripper range: 0.035m - 0.070m")
        self.get_logger().info(f"🎯 Base frame: {self.base_link_frame}")
        self.get_logger().info(f"🔄 Update rate: {self.update_rate} Hz")

    def initialize_collision_objects(self):
        """Initialize collision objects by clearing any existing ones"""
        self.get_logger().info("Initializing collision objects...")
        
        # Clear any existing collision objects
        for obj_id in self.collision_ids.values():
            remove_obj = self.create_collision_object(obj_id, Pose(), [], CollisionObject.REMOVE)
            self.collision_object_pub.publish(remove_obj)
        
        # Small delay to ensure removal is processed
        import time
        time.sleep(0.5)

    def gripper_position_callback(self, msg: Float32):
        """
        Callback for gripper position updates from OnRobot driver.
        """
        with self.lock:
            old_width = self.current_width
            self.current_width = msg.data
            
            # Log significant changes
            if abs(old_width - self.current_width) > 0.001:
                self.get_logger().debug(
                    f"Gripper position updated: {self.current_width:.4f}m"
                )

    def calculate_finger_positions(self, total_width: float) -> Dict[str, float]:
        """
        Calculate finger positions based on total opening width.
        Uses the asymmetric configuration from your URDF.
        """
        # Calculate finger displacement from closed position
        # Closed: 0.035m opening, Open: 0.070m opening  
        # Each finger moves 0.0175m from closed to open
        finger_displacement = (total_width - 0.035) / 2.0
        
        # World coordinates based on URDF initial positions
        left_finger_x = self.left_finger_initial_x - finger_displacement
        right_finger_x = self.right_finger_initial_x + finger_displacement
        
        # Joint space coordinates (for controller)
        left_joint = finger_displacement  # 0.0 to 0.0175
        right_joint = -finger_displacement  # 0.0 to -0.0175
        
        return {
            'left_world_x': left_finger_x,
            'right_world_x': right_finger_x,
            'left_joint': left_joint,
            'right_joint': right_joint,
            'finger_displacement': finger_displacement,
            'opening_center_x': (left_finger_x + right_finger_x) / 2.0,
            'opening_width': abs(left_finger_x - right_finger_x) - self.finger_thickness
        }

    def create_collision_object(self, object_id: str, pose: Pose, 
                              dimensions: list, operation: int) -> CollisionObject:
        """
        Create a collision object with the specified parameters.
        """
        collision_object = CollisionObject()
        collision_object.header = Header()
        collision_object.header.stamp = self.get_clock().now().to_msg()
        collision_object.header.frame_id = self.base_link_frame
        collision_object.id = object_id
        collision_object.operation = operation
        
        if operation == CollisionObject.ADD:
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = dimensions
            
            collision_object.primitives.append(primitive)
            collision_object.primitive_poses.append(pose)
        
        return collision_object

    def update_collision_objects(self):
        """
        Update all collision objects based on current gripper position.
        """
        try:
            with self.lock:
                current_width = self.current_width
            
            # Calculate current finger positions
            positions = self.calculate_finger_positions(current_width)
            
            # Update left finger collision
            self.update_finger_collision(
                'left_finger', 
                positions['left_world_x'],
                positions['left_joint']
            )
            
            # Update right finger collision  
            self.update_finger_collision(
                'right_finger',
                positions['right_world_x'], 
                positions['right_joint']
            )
            
            # Update opening space collision
            self.update_opening_space(positions)
            
            # Publish visualization markers if enabled
            if self.publish_markers:
                self.publish_visualization_markers(positions)
                
        except Exception as e:
            self.get_logger().error(f"Error updating collision objects: {e}")

    def update_finger_collision(self, finger_type: str, world_x: float, joint_pos: float):
        """
        Update collision object for a single finger.
        """
        # Remove previous collision object
        remove_obj = self.create_collision_object(
            self.collision_ids[finger_type],
            Pose(),  # Dummy pose for remove
            [],
            CollisionObject.REMOVE
        )
        self.collision_object_pub.publish(remove_obj)
        
        # Add updated collision object
        pose = Pose()
        pose.position.x = world_x
        pose.position.y = self.finger_y_position
        pose.position.z = self.finger_z_position
        pose.orientation.w = 1.0
        
        dimensions = [
            self.finger_thickness,  # X - thickness
            self.finger_height,     # Y - height  
            self.finger_length      # Z - length
        ]
        
        add_obj = self.create_collision_object(
            self.collision_ids[finger_type],
            pose,
            dimensions,
            CollisionObject.ADD
        )
        
        self.collision_object_pub.publish(add_obj)

    def update_opening_space(self, positions: Dict[str, float]):
        """
        Update collision object for the space between fingers.
        """
        # Remove previous opening space
        remove_obj = self.create_collision_object(
            self.collision_ids['gripper_opening'],
            Pose(),  # Dummy pose
            [],
            CollisionObject.REMOVE
        )
        self.collision_object_pub.publish(remove_obj)
        
        # Only add opening space if there's actually an opening
        if positions['opening_width'] > 0.001:
            # Add updated opening space
            pose = Pose()
            pose.position.x = positions['opening_center_x']
            pose.position.y = self.finger_y_position
            pose.position.z = self.finger_z_position
            pose.orientation.w = 1.0
            
            # Dimensions for opening space
            dimensions = [
                0.02,  # X - depth
                max(0.001, positions['opening_width']),  # Y - width (ensure positive)
                0.02   # Z - height
            ]
            
            add_obj = self.create_collision_object(
                self.collision_ids['gripper_opening'],
                pose,
                dimensions,
                CollisionObject.ADD
            )
            
            self.collision_object_pub.publish(add_obj)

    def publish_visualization_markers(self, positions: Dict[str, float]):
        """
        Publish visualization markers for debugging and visualization.
        """
        marker_array = MarkerArray()
        timestamp = self.get_clock().now().to_msg()
        
        # Left finger marker
        left_marker = Marker()
        left_marker.header.frame_id = self.base_link_frame
        left_marker.header.stamp = timestamp
        left_marker.ns = "gripper_fingers"
        left_marker.id = 0
        left_marker.type = Marker.CUBE
        left_marker.action = Marker.ADD
        
        left_marker.pose.position.x = positions['left_world_x']
        left_marker.pose.position.y = self.finger_y_position
        left_marker.pose.position.z = self.finger_z_position
        left_marker.pose.orientation.w = 1.0
        
        left_marker.scale.x = self.finger_thickness
        left_marker.scale.y = self.finger_height
        left_marker.scale.z = self.finger_length
        
        left_marker.color.r = 0.0
        left_marker.color.g = 0.8
        left_marker.color.b = 0.0
        left_marker.color.a = 0.6
        
        marker_array.markers.append(left_marker)
        
        # Right finger marker
        right_marker = Marker()
        right_marker.header.frame_id = self.base_link_frame
        right_marker.header.stamp = timestamp
        right_marker.ns = "gripper_fingers"
        right_marker.id = 1
        right_marker.type = Marker.CUBE
        right_marker.action = Marker.ADD
        
        right_marker.pose.position.x = positions['right_world_x']
        right_marker.pose.position.y = self.finger_y_position
        right_marker.pose.position.z = self.finger_z_position
        right_marker.pose.orientation.w = 1.0
        
        right_marker.scale.x = self.finger_thickness
        right_marker.scale.y = self.finger_height
        right_marker.scale.z = self.finger_length
        
        right_marker.color.r = 0.8
        right_marker.color.g = 0.0
        right_marker.color.b = 0.0
        right_marker.color.a = 0.6
        
        marker_array.markers.append(right_marker)
        
        # Opening space marker (only if there's an opening)
        if positions['opening_width'] > 0.001:
            opening_marker = Marker()
            opening_marker.header.frame_id = self.base_link_frame
            opening_marker.header.stamp = timestamp
            opening_marker.ns = "gripper_opening"
            opening_marker.id = 2
            opening_marker.type = Marker.CUBE
            opening_marker.action = Marker.ADD
            
            opening_marker.pose.position.x = positions['opening_center_x']
            opening_marker.pose.position.y = self.finger_y_position
            opening_marker.pose.position.z = self.finger_z_position
            opening_marker.pose.orientation.w = 1.0
            
            opening_marker.scale.x = 0.02
            opening_marker.scale.y = max(0.001, positions['opening_width'])
            opening_marker.scale.z = 0.02
            
            opening_marker.color.r = 0.0
            opening_marker.color.g = 0.0
            opening_marker.color.b = 0.8
            opening_marker.color.a = 0.3
            
            marker_array.markers.append(opening_marker)
        
        # Text marker showing current width
        text_marker = Marker()
        text_marker.header.frame_id = self.base_link_frame
        text_marker.header.stamp = timestamp
        text_marker.ns = "gripper_info"
        text_marker.id = 3
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        
        text_marker.pose.position.x = positions['opening_center_x']
        text_marker.pose.position.y = self.finger_y_position + 0.02
        text_marker.pose.position.z = self.finger_z_position + 0.03
        text_marker.pose.orientation.w = 1.0
        
        text_marker.scale.z = 0.01  # Text height
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0
        
        text_marker.text = f"Width: {self.current_width*1000:.1f}mm"
        
        marker_array.markers.append(text_marker)
        
        self.marker_pub.publish(marker_array)

    async def control_gripper(self, position: float, force: float = 50.0, 
                            timeout: float = 10.0) -> bool:
        """
        Control gripper via OnRobot driver action server.
        This can be called from MoveIt actions or other nodes.
        """
        self.get_logger().info(
            f"Controlling gripper: position={position:.3f}m, force={force:.1f}N"
        )
        
        if not self.gripper_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Gripper action server not available")
            return False
        
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = float(position)
        goal_msg.command.max_effort = float(force)
        
        try:
            future = self.gripper_action_client.send_goal_async(goal_msg)
            await future
            
            if future.result() is not None:
                goal_handle = future.result()
                if goal_handle.accepted:
                    self.get_logger().info("Gripper goal accepted")
                    
                    # Wait for result
                    result_future = goal_handle.get_result_async()
                    await result_future
                    
                    if result_future.result() is not None:
                        result = result_future.result().result
                        self.get_logger().info(
                            f"Gripper action completed: "
                            f"reached_goal={result.reached_goal}, "
                            f"stalled={result.stalled}"
                        )
                        return result.reached_goal
                    
            return False
            
        except Exception as e:
            self.get_logger().error(f"Error controlling gripper: {e}")
            return False

    def emergency_stop(self):
        """
        Emergency stop function for the gripper.
        """
        self.get_logger().warn("🛑 Emergency stop triggered for gripper")
        # Send stop command to gripper
        stop_goal = GripperCommand.Goal()
        stop_goal.command.position = self.current_width  # Maintain current position
        stop_goal.command.max_effort = 0.0  # Zero force
        
        if self.gripper_action_client.wait_for_server(timeout_sec=1.0):
            future = self.gripper_action_client.send_goal_async(stop_goal)
            self.get_logger().info("Emergency stop command sent to gripper")

    def get_gripper_status(self) -> Dict[str, Any]:
        """
        Get current gripper status for monitoring and diagnostics.
        """
        with self.lock:
            positions = self.calculate_finger_positions(self.current_width)
            
            return {
                'current_width': self.current_width,
                'target_width': self.target_width,
                'is_moving': self.is_moving,
                'left_finger_position': positions['left_joint'],
                'right_finger_position': positions['right_joint'],
                'left_finger_world_x': positions['left_world_x'],
                'right_finger_world_x': positions['right_world_x'],
                'opening_width': positions['opening_width'],
                'frame_id': self.base_link_frame
            }


def main():
    rclpy.init()
    
    try:
        bridge = MoveItGripperBridge()
        
        # Use multi-threaded executor for better performance
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(bridge)
        
        bridge.get_logger().info("🚀 MoveIt Gripper Bridge started")
        bridge.get_logger().info("📊 Publishing collision objects and markers")
        
        executor.spin()
        
    except KeyboardInterrupt:
        bridge.get_logger().info("🛑 Shutting down MoveIt Gripper Bridge")
    except Exception as e:
        bridge.get_logger().error(f"💥 Error in MoveIt Gripper Bridge: {e}")
    finally:
        if 'bridge' in locals():
            bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()