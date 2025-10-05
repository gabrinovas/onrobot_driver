from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_path = FindPackageShare('onrobot_driver').find('onrobot_driver')
    
    return LaunchDescription([
        # Mock server for simulation
        ExecuteProcess(
            cmd=['python3', os.path.join(pkg_path, 'test/mock_gripper_server.py')],
            output='screen'
        ),
        
        # Driver node with simulation config
        Node(
            package='onrobot_driver',
            executable='onrobot_driver_node',
            name='onrobot_driver',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('onrobot_driver'),
                    'config',
                    'simulation_params.yaml'
                ])
            ]
        ),
    ])