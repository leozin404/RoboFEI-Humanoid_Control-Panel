from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    repo_dir = DeclareLaunchArgument(
        "repo_dir",
        default_value="~/RoboFEI-HT_2023_SOFTWARE",
        description="Team repo checkout; processes started by the panel run from here",
    )
    imu_command = DeclareLaunchArgument(
        "imu_command",
        default_value="ros2 run um7 um7_node",
        description="Command that starts the IMU node",
    )
    team_number = DeclareLaunchArgument(
        "team_number",
        default_value="7",
        description="GameController team number passed to gamecontroller_bringup",
    )

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
        parameters=[{
            "repo_dir": LaunchConfiguration("repo_dir"),
            "imu_command": LaunchConfiguration("imu_command"),
        }],
    )

    panel_teleop_node = Node(
        package="control_panel_pkg",
        executable="panel_teleop_node",
        name="panel_teleop_node",
        output="screen",
        parameters=[{
            "repo_dir": LaunchConfiguration("repo_dir"),
            "team_number": LaunchConfiguration("team_number"),
        }],
    )

    return LaunchDescription([
        repo_dir,
        imu_command,
        team_number,
        usb_monitor_node,
        system_status_node,
        panel_teleop_node,
    ])
