# OnRobot Gripper Driver for ROS2 Humble

A versatile ROS2 Python driver for controlling various OnRobot grippers via the Compute Box. Supports multiple gripper models including 2FG7, 3FG15, VGC10, and more.

## Features

-   `Multi-Gripper Support`: Compatible with various OnRobot gripper models (2FG7, 3FG15, VGC10, RG2, etc.)
-   `ROS2 Humble Compatibility`: Modern ROS2 Python implementation
-   `Compute Box Integration`: Communicates via Ethernet with OnRobot Compute Box (192.168.1.1)
- `Standard Interfaces`:
    - GripperCommand action interface
    - Joint state publishing
    - Real-time status monitoring
- `Configurable Parameters`: Easy configuration for different gripper models
- `Extensible Architecture`: Easy to add support for new gripper models

## Prerequisites

### Hardware Requirements
- OnRobot gripper (2FG7, 3FG15, VGC10, etc.)
- OnRobot Compute Box
- Ethernet connection between Compute Box and ROS2 machine
- Gripper properly connected to Compute Box

### Software Requirements
- ROS2 Humble Hawksbill
- Python 3.8+
- Linux (Ubuntu 22.04 recommended)

## Network Configuration
The Compute Box uses static IP `192.168.1.1`. Configure your ROS2 machine to be on the same subnet:

## Installation

### 1. Clone the Repository
```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/your-username/onrobot_driver.git
```

### 2. Install WSL and set WSL2 as default:
```
sudo apt update
sudo apt install python3-pip
pip3 install python-socketcan
```

### 3. Restart your computer when prompted.
```
cd ~/ros2_ws
colcon build --packages-select onrobot_driver
source install/setup.bash
```

## Usage
### Basic Launch (Default Gripper)
```
ros2 launch onrobot_driver onrobot_driver.launch.py
```

### Launch with Specific Gripper
```
# For 2FG7 gripper
ros2 launch onrobot_driver onrobot_driver.launch.py gripper_type:=2FG7

# For 3FG15 gripper  
ros2 launch onrobot_driver onrobot_driver.launch.py gripper_type:=3FG15 max_width:=0.15

# For VGC10 vacuum gripper
ros2 launch onrobot_driver onrobot_driver.launch.py gripper_type:=VGC10
```

### Custom IP Address
If your Compute Box has a different IP:
```
ros2 launch onrobot_driver onrobot_driver.launch.py ip_address:=192.168.1.50
```


## Controlling the Gripper

### Using Action Interface
```
# Open gripper to 40mm with 50N force
ros2 action send_goal gripper_action control_msgs/action/GripperCommand "{
  command: {position: 0.04, max_effort: 50.0}
}"

# Close gripper completely with maximum force
ros2 action send_goal gripper_action control_msgs/action/GripperCommand "{
  command: {position: 0.0, max_effort: 100.0}
}"
```

### Using Python Script
```
import rclpy
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand

def control_gripper():
    rclpy.init()
    node = rclpy.create_node('gripper_client')
    client = ActionClient(node, GripperCommand, 'gripper_action')
    
    goal_msg = GripperCommand.Goal()
    goal_msg.command.position = 0.03  # 30mm opening
    goal_msg.command.max_effort = 40.0  # 40N force
    
    client.wait_for_server()
    future = client.send_goal_async(goal_msg)
    rclpy.spin_until_future_complete(node, future)
    
    node.destroy_node()
    rclpy.shutdown()
```

## Configuration

### Parameter File
Edit `config/onrobot_params.yaml` for permanent configuration:

```
/**:
  ros__parameters:
    gripper_type: "2FG7"           # Gripper model
    ip_address: "192.168.1.1"      # Compute Box IP
    port: 502                      # Modbus TCP port
    max_width: 0.070               # Maximum opening (70mm for 2FG7)
    min_width: 0.0                 # Minimum opening
    max_force: 70.0                # Maximum force (70N for 2FG7)
    update_rate: 100.0             # Control loop frequency
    use_sim_time: false            # Simulation mode
```

## Gripper-Specific Parameters
### 2FG7 Configuration
```
gripper_type: "2FG7"
max_width: 0.070    # 70mm
max_force: 70.0     # 70N
min_width: 0.0
```

### 3FG15 Configuration
```
gripper_type: "3FG15"
max_width: 0.150    # 150mm
max_force: 90.0     # 90N
min_width: 0.0
```

### VGC10 Configuration
```
gripper_type: "VGC10"
max_width: 0.0      # Not applicable for vacuum
max_force: 70.0     # 70N vacuum force
min_width: 0.0
```

## ROS2 Topics and Services
### Published Topics
- `/joint_states` (`sensor_msgs/JointState`) - Gripper position and force
- `/gripper_status` (`std_msgs/Bool`) - Gripper ready status
- `/gripper_position` (`std_msgs/Float32`) - Current position

### Action Services
- `/gripper_action` (`control_msgs/GripperCommand`) - Current position

### Parameters
- `gripper_type` - Gripper model identifier
- `ip_address` - Compute Box IP address
- `max_width` - Maximum opening width in meters
- `max_force` - Maximum force in Newtons

## Troubleshooting
### Common Issues
1. Connection Timeout
```
ERROR: Failed to connect to 192.168.1.1:502
```
- Verify network connectivity: `ping 192.168.1.1`
- Check Ethernet cable and Compute Box power
- Ensure correct IP configuration


2. Gripper Not Responding
```
WARNING: No response from gripper
```
- Check gripper power and connections to Compute Box
- Verify gripper type parameter matches physical gripper
- Check Compute Box status LEDs

3. Incorrect Position Readings
```
Position readings seem inaccurate
```
- Verify `max_width` parameter matches your gripper model
- Check for mechanical obstructions
- Calibrate gripper if supported by model

### Debug Mode
Enable debug output for detailed logging:
```
ros2 launch onrobot_driver onrobot_driver.launch.py --log-level debug
```

## Extending for New Grippers

### 1. Add Gripper Configuration

Update the gripper configuration in your launch file or parameters
```
# For a new gripper model (e.g., "RG6")
parameters={
    'gripper_type': 'RG6',
    'max_width': 0.160,    # 160mm
    'max_force': 235.0,    # 235N
    'min_width': 0.0,
}
```

### 2. Custom Command Protocol (If Needed)
If a new gripper uses a different communication protocol, extend the driver:
```
# In onrobot_driver/drivers/onrobot_gripper.py
def create_command_bytes(self, position: int, force: int) -> bytes:
    if self.gripper_type == "NEW_MODEL":
        # Custom protocol for new gripper
        return self._create_new_model_command(position, force)
    else:
        # Standard protocol
        return self._create_standard_command(position, force)
```

### 3. Add Model-Specific Logic
Override methods for special behavior:
```
def pre_process_command(self, position: float, force: float) -> tuple:
    """Add model-specific command preprocessing"""
    if self.gripper_type == "VGC10":
        # Vacuum gripper special handling
        return self._process_vacuum_command(position, force)
    else:
        return position, force
```

## API Reference

### OnRobotGripper Class

#### Core Methods
- `move_to_position(position, force)` - Move gripper to specified position
- `read_status()` - Read current gripper status
- `connect()` - Establish connection to Compute Box
- `disconnect()` - Close connection

#### Configuration Methods
- `set_gripper_type(model)` - Set gripper model
- `set_force_limit(force)` - Set maximum force
- `set_speed(speed)` - Set movement speed

### Contributing 
We welcome contributions for supporting additional OnRobot gripper models!

### Adding New Gripper Support
1. Fork the repository
2. Test with your gripper model
3. Add configuration parameters for your gripper
4. Submit a pull request with:
    - Gripper specifications
    - Test results
    - Updated documentation

### Testing New Grippers
```
# Test script for new gripper models
ros2 launch onrobot_driver onrobot_driver.launch.py gripper_type:=NEW_MODEL
ros2 action send_goal gripper_action control_msgs/action/GripperCommand "{command: {position: 0.02, max_effort: 30.0}}"
```
### License
This project is licensed under the Apache 2.0 License - see the LICENSE file for details.

### Support
- `Issues`: Use the GitHub issue tracker
- `Documentation`: OnRobot Official Documentation
- `Community`: ROS Discourse and OnRobot user forums

### Acknowledgments
- OnRobot for providing excellent gripper hardware and documentation
- ROS2 community for robust robotics framework
- Contributors who help expand gripper model support

