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
        # Start the mock gripper server for simulation testing.
        # This simulates the OnRobot Compute Box so the driver can be tested without hardware.
        ExecuteProcess(
            cmd=['python3', os.path.join(onrobot_driver_share, 'test/mock_gripper_server.py')],
            output='screen'
        ),
        
        # Start the driver node with simulation parameters after a short delay (2 seconds).
        # This ensures the mock server is running before the driver starts.
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='onrobot_driver',
                    executable='onrobot_driver_node',
                    name='onrobot_driver',
                    output='screen',
                    parameters=[{
                        'gripper_type': '2FG7',         # Gripper model type
                        'ip_address': '127.0.0.1',      # Localhost for simulation
                        'port': 1502,                   # Non-standard port for simulation
                        'max_width': 0.085,             # Max opening width in meters
                        'min_width': 0.0,               # Min opening width in meters
                        'max_force': 100.0,             # Max gripping force in Newtons
                        'update_rate': 10.0,            # Update rate in Hz
                        'simulation_mode': True,        # Enable simulation mode
                    }]
                )
            ]
        ),
        
        # Run the integration test after everything is started (5 seconds delay).
        # This script will test the driver node's functionality in the simulated environment.
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