from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Driver node with simulation config
        Node(
            package='onrobot_driver',
            executable='onrobot_driver_node',
            name='onrobot_driver',
            output='screen',
            parameters=[{
                'gripper_type': '2FG7',
                'ip_address': '127.0.0.1',  # Use localhost to force simulation
                'port': 502,
                'max_width': 0.085,
                'min_width': 0.0,
                'max_force': 100.0,
                'update_rate': 10.0,
                'simulation_mode': True,  # Force simulation mode
            }]
        ),
    ])