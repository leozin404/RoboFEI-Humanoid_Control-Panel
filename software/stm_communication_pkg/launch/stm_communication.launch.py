from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port_arg = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/stm_panel",
        description="udev symlink for the STM32 control panel USB CDC device",
    )

    stm_communication_node = Node(
        package="stm_communication_pkg",
        executable="stm_communication_node",
        name="stm_communication_node",
        output="screen",
        parameters=[{"serial_port": LaunchConfiguration("serial_port")}],
    )

    return LaunchDescription([
        serial_port_arg,
        stm_communication_node,
    ])
