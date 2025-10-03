from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim',
            default_value='true',
            description='Use simulation (no hardware connection)'
        ),
        
        DeclareLaunchArgument(
            'ip_address',
            default_value='127.0.0.1',  # Localhost for simulation
            description='IP address for simulation'
        ),
        
        Node(
            package='onrobot_driver',
            executable='onrobot_driver_node',
            name='onrobot_driver',
            output='screen',
            parameters=[{
                'gripper_type': '2FG7',
                'ip_address': LaunchConfiguration('ip_address'),
                'port': 502,
                'max_width': 0.085,
                'min_width': 0.0,
                'max_force': 100.0,
                'update_rate': 100.0,
            }]
        ),
        
        # You can add a simple controller for testing
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen',
        ),
    ])