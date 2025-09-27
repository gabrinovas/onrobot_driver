#!/usr/bin/env python3

from onrobot_driver.hardware.onrobot_gripper_hardware_interface import OnRobotGripperHardwareInterface

def main():
    # This function is required for the plugin system
    pass

# Factory function for plugin system
def create_hardware_interface():
    return OnRobotGripperHardwareInterface()