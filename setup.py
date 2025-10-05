from setuptools import setup
import os
from glob import glob

package_name = 'onrobot_driver'

setup(
    name=package_name,
    version='1.0.0',
    packages=[
        package_name, 
        f'{package_name}.nodes', 
        f'{package_name}.drivers',
        f'{package_name}.hardware'
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
        # Install test files
        (os.path.join('share', package_name, 'test'),
         glob('test/*.py')),
        # Install the new control script
        (os.path.join('lib', package_name),
         glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gabriel Novas',
    maintainer_email='gabriel.novas@aimen.es',
    description='ROS2 Python driver for OnRobot grippers via Compute Box',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'onrobot_driver_node = onrobot_driver.nodes.onrobot_driver_node:main',
            'control_gripper = onrobot_driver.scripts.control_gripper:main',
        ],
    },
)