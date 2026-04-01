#!/usr/bin/env python3
"""
Launch file for the BNO055 IMU calibration node.

Starts the bno055_calib_node which:
  - Runs the BNO055 in NDOF (9-DOF fusion) mode
  - Prints calibration status and movement guidance to the terminal at 1 Hz
  - Publishes diagnostic_msgs/DiagnosticArray on ~/diagnostics at 1 Hz
  - Provides ~/save_calibration (std_srvs/Trigger) to persist offsets to YAML
  - Provides ~/load_calibration (std_srvs/Trigger) to apply saved offsets

Example usage:
    ros2 launch bno055_hardware_interface bno055_calib.launch.py
    ros2 launch bno055_hardware_interface bno055_calib.launch.py \\
        i2c_bus:=1 i2c_addr:=40

    # In another terminal — save calibration offsets to file:
    ros2 service call /bno055_calib_node/save_calibration std_srvs/srv/Trigger

    # Load previously saved offsets back onto the sensor:
    ros2 service call /bno055_calib_node/load_calibration std_srvs/srv/Trigger
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'i2c_bus',
            default_value='1',
            description='I2C bus number (e.g. 1 → /dev/i2c-1)',
        ),
        DeclareLaunchArgument(
            'i2c_addr',
            default_value='40',   # 0x28 decimal
            description='I2C device address as a decimal integer (default: 40 = 0x28)',
        ),
        DeclareLaunchArgument(
            'calib_file',
            default_value=os.path.expanduser('~/.ros/bno055_calib.yaml'),
            description='Path to the calibration profile YAML file',
        ),
    ]

    calib_node = Node(
        package='bno055_hardware_interface',
        executable='bno055_calib_node',
        name='bno055_calib_node',
        output='screen',
        parameters=[{
            'i2c_bus':    LaunchConfiguration('i2c_bus'),
            'i2c_addr':   LaunchConfiguration('i2c_addr'),
            'calib_file': LaunchConfiguration('calib_file'),
        }],
    )

    return LaunchDescription(declared_arguments + [calib_node])
