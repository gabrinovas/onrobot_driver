from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Define launch arguments
    gripper_type_arg = DeclareLaunchArgument(
        'gripper_type',
        default_value='2FG7',
        description='Type of OnRobot gripper (2FG7, 3FG15, VGC10, etc.)'
    )
    
    ip_address_arg = DeclareLaunchArgument(
        'ip_address',
        default_value='192.168.1.1',
        description='IP address of OnRobot Compute Box'
    )
    
    max_width_arg = DeclareLaunchArgument(
        'max_width',
        default_value='0.085',
        description='Maximum gripper width in meters'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='onrobot_params.yaml',
        description='Path to config file'
    )
    
    # Node configuration
    onrobot_driver_node = Node(
        package='onrobot_driver',
        executable='onrobot_driver_node',
        name='onrobot_driver',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('onrobot_driver'),
                'config',
                LaunchConfiguration('config_file')
            ]),
            {
                'gripper_type': LaunchConfiguration('gripper_type'),
                'ip_address': LaunchConfiguration('ip_address'),
                'max_width': LaunchConfiguration('max_width'),
            }
        ],
        remappings=[
            # You can add remappings here if needed
            # ('gripper_action', 'onrobot_gripper_action'),
        ]
    )
    
    return LaunchDescription([
        gripper_type_arg,
        ip_address_arg,
        max_width_arg,
        config_file_arg,
        onrobot_driver_node,
    ])