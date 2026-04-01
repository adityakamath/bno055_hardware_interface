#!/usr/bin/env python3
"""
Launch file for the BNO055 IMU hardware interface.

Starts the complete ros2_control stack for the BNO055 IMU, including:
- Robot state publisher for TF transforms
- Controller manager with the BNO055 SensorInterface hardware plugin
- IMU sensor broadcaster publishing sensor_msgs/Imu to /bno055/imu

Example usage:
    ros2 launch bno055_hardware_interface bno055.launch.py
    ros2 launch bno055_hardware_interface bno055.launch.py i2c_bus:=1 i2c_addr:=28
    ros2 launch bno055_hardware_interface bno055.launch.py axis_remap:=P2
    ros2 launch bno055_hardware_interface bno055.launch.py enable_mock:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'i2c_bus',
            default_value='1',
            description='I2C bus number (e.g. 1 → /dev/i2c-1)'
        ),
        DeclareLaunchArgument(
            'i2c_addr',
            default_value='28',
            description='I2C device address in hex without 0x prefix (default: 28 = 0x28)'
        ),
        DeclareLaunchArgument(
            'axis_remap',
            default_value='P1',
            description='BNO055 axis placement configuration: P0-P7 (see datasheet Table 3-4)'
        ),
        DeclareLaunchArgument(
            'enable_mock',
            default_value='false',
            description='Use mock/simulation mode (no hardware required)'
        ),
    ]

    i2c_bus = LaunchConfiguration('i2c_bus')
    i2c_addr = LaunchConfiguration('i2c_addr')
    axis_remap = LaunchConfiguration('axis_remap')
    enable_mock = LaunchConfiguration('enable_mock')

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('bno055_hardware_interface'), 'config', 'bno055.urdf.xacro']
            ),
            ' ',
            'i2c_bus:=', i2c_bus,
            ' ',
            'i2c_addr:=', i2c_addr,
            ' ',
            'axis_remap:=', axis_remap,
            ' ',
            'enable_mock:=', enable_mock,
        ]
    )
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    # Controller configuration
    controller_config = PathJoinSubstitution(
        [FindPackageShare('bno055_hardware_interface'), 'config', 'imu_broadcaster.yaml']
    )

    # Controller manager (ros2_control_node)
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='both',
        parameters=[robot_description, controller_config],
    )

    # IMU sensor broadcaster spawner
    imu_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['imu_sensor_broadcaster', '--controller-manager', '/controller_manager'],
    )

    return LaunchDescription(
        declared_arguments + [
            robot_state_publisher_node,
            controller_manager_node,
            imu_broadcaster_spawner,
        ]
    )
