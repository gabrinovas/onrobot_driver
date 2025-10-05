from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Get the package share directory
    onrobot_driver_share = FindPackageShare('onrobot_driver').find('onrobot_driver')
    
    return LaunchDescription([
        # Mock server for simulation testing
        ExecuteProcess(
            cmd=['python3', os.path.join(onrobot_driver_share, 'test/mock_gripper_server.py')],
            output='screen'
        ),
        
        # Driver node with simulation config
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='onrobot_driver',
                    executable='onrobot_driver_node',
                    name='onrobot_driver',
                    output='screen',
                    parameters=[{
                        'gripper_type': '2FG7',
                        'ip_address': '127.0.0.1',
                        'port': 1502,
                        'max_width': 0.085,
                        'min_width': 0.0,
                        'max_force': 100.0,
                        'update_rate': 10.0,
                        'simulation_mode': True,
                    }]
                )
            ]
        ),
        
        # Run integration test after everything is started
        TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    cmd=['python3', os.path.join(onrobot_driver_share, 'test/integration_test.py'), '--no-mock'],
                    output='screen'
                )
            ]
        ),
    ])