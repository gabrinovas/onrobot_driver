from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # This launch file starts the OnRobot gripper driver node with configurable parameters.
    # The configuration file can be specified via the 'config_file' launch argument.
    return LaunchDescription([
        # Declare a launch argument for the config file (default: onrobot_params.yaml)
        DeclareLaunchArgument(
            'config_file',
            default_value='onrobot_params.yaml',
            description='Path to config file'
        ),
        
        # Launch the onrobot_driver_node with the specified parameters file
        Node(
            package='onrobot_driver',
            executable='onrobot_driver_node',
            name='onrobot_driver',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('onrobot_driver'),
                    'config',
                    LaunchConfiguration('config_file')
                ])
            ]
        ),
    ])