#!/usr/bin/env python3

import os
import sys

def check_build():
    print("Verifying OnRobot Driver Build")
    print("=" * 50)
    
    # Check if package is in ROS2 path
    try:
        import subprocess
        result = subprocess.run(['ros2', 'pkg', 'list'], capture_output=True, text=True)
        if 'onrobot_driver' in result.stdout:
            print("✅ onrobot_driver package found in ROS2 packages")
        else:
            print("❌ onrobot_driver package NOT found in ROS2 packages")
            return False
    except Exception as e:
        print(f"❌ Error checking ROS2 packages: {e}")
        return False
    
    # Check if node executable exists
    node_path = os.path.expanduser('~/install/onrobot_driver/lib/onrobot_driver/onrobot_driver_node')
    if os.path.exists(node_path):
        print(f"✅ Node executable found: {node_path}")
        # Check if it's executable
        if os.access(node_path, os.X_OK):
            print("✅ Node executable is executable")
        else:
            print("❌ Node executable is NOT executable")
            return False
    else:
        print(f"❌ Node executable NOT found: {node_path}")
        # List what's in the directory
        lib_dir = os.path.dirname(node_path)
        if os.path.exists(lib_dir):
            print(f"Contents of {lib_dir}:")
            for item in os.listdir(lib_dir):
                print(f"  - {item}")
        return False
    
    # Check if Python modules are installed
    python_paths = [
        '~/install/onrobot_driver/lib/python3.10/site-packages/onrobot_driver',
        '~/install/onrobot_driver/local/lib/python3.10/site-packages/onrobot_driver'
    ]
    
    found_python = False
    for path in python_paths:
        full_path = os.path.expanduser(path)
        if os.path.exists(full_path):
            print(f"✅ Python package found: {full_path}")
            found_python = True
            # List contents
            print("Python package contents:")
            for root, dirs, files in os.walk(full_path):
                for file in files:
                    if file.endswith('.py'):
                        rel_path = os.path.relpath(os.path.join(root, file), full_path)
                        print(f"  - {rel_path}")
            break
    
    if not found_python:
        print("❌ Python package not found in expected locations")
        # Try to find it
        install_base = os.path.expanduser('~/install/onrobot_driver')
        for root, dirs, files in os.walk(install_base):
            if 'onrobot_driver' in dirs and any(f.endswith('.py') for f in files):
                print(f"Found potential Python package at: {root}")
    
    return True

if __name__ == '__main__':
    success = check_build()
    print(f"\nBuild verification: {'SUCCESS' if success else 'FAILED'}")
    sys.exit(0 if success else 1)