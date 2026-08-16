from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    usb_monitor_node = Node(
        package="control_panel_pkg",
        executable="usb_monitor_node",
        name="usb_monitor_node",
        output="screen",
    )

    system_status_node = Node(
        package="control_panel_pkg",
        executable="system_status_node",
        name="system_status_node",
        output="screen",
    )

    panel_teleop_node = Node(
        package="control_panel_pkg",
        executable="panel_teleop_node",
        name="panel_teleop_node",
        output="screen",
    )

    return LaunchDescription([
        usb_monitor_node,
        system_status_node,
        panel_teleop_node,
    ])
