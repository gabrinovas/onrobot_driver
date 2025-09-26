from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # This launch file includes the main launch file with package-specific configurations
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'gripper_type',
            default_value='2FG7',
            description='Type of OnRobot gripper'
        ),
        
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        
        # Include the main launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('onrobot_driver'),
                    'launch',
                    'onrobot_driver.launch.py'
                ])
            ]),
            launch_arguments={
                'gripper_type': LaunchConfiguration('gripper_type'),
                'ip_address': '192.168.1.1',  # Default Compute Box IP
                'max_width': '0.085',
                'config_file': 'onrobot_params.yaml'
            }.items()
        )
    ])