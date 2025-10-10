from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # This launch file starts the OnRobot gripper driver node with hardcoded parameters
    # for a simple test or simulation. It uses localhost and enables simulation mode.
    return LaunchDescription([
        Node(
            package='onrobot_driver',              # ROS2 package name
            executable='onrobot_driver_node',      # Executable node name
            name='onrobot_driver',                 # Node name
            output='screen',                       # Output logs to screen
            parameters=[{                          # Inline parameters for the node
                'gripper_type': '2FG7',            # Gripper model type
                'ip_address': '127.0.0.1',         # Localhost for simulation
                'port': 502,                       # TCP port (default Modbus)
                'max_width': 0.085,                # Max opening width in meters
                'min_width': 0.0,                  # Min opening width in meters
                'max_force': 100.0,                # Max gripping force in Newtons
                'update_rate': 10.0,               # Update rate in Hz
                'simulation_mode': True,           # Enable simulation mode
            }]
        ),
    ])