#!/usr/bin/env python3

from onrobot_driver.hardware.onrobot_gripper_hardware_interface import OnRobotGripperHardwareInterface

# Factory function for plugin system
def get_hardware_interface():
    return OnRobotGripperHardwareInterface()