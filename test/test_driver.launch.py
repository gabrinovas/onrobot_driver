from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
import os

def generate_launch_description():
    # Start mock server first
    mock_server = ExecuteProcess(
        cmd=['python3', os.path.join(os.getcwd(), 'test/mock_gripper_server.py')],
        output='screen'
    )
    
    # Start driver node after a delay
    driver_node = Node(
        package='onrobot_driver',
        executable='onrobot_driver_node',
        name='onrobot_driver',
        output='screen',
        parameters=[os.path.join(os.getcwd(), 'config/test_params.yaml')]
    )
    
    # Delay driver start to ensure mock server is ready
    delayed_driver = TimerAction(
        period=2.0,
        actions=[driver_node]
    )
    
    return LaunchDescription([
        mock_server,
        delayed_driver,
    ])