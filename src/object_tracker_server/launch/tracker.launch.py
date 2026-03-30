"""
Launch the object tracker action server and (optionally) the Waveshare motor driver.

Usage:
  ros2 launch object_tracker_server tracker.launch.py
  ros2 launch object_tracker_server tracker.launch.py launch_motor_driver:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    launch_motor_arg = DeclareLaunchArgument(
        "launch_motor_driver",
        default_value="false",
        description="Set to true to also launch the Waveshare motor driver node.",
    )

    tracker_server_node = Node(
        package="object_tracker_server",
        executable="object_tracker_server",
        name="object_tracker_server",
        output="screen",
        emulate_tty=True,
    )

    motor_driver_node = Node(
        package="waveshare_motor_driver",
        executable="waveshare_motor_driver",
        name="waveshare_motor_driver",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("launch_motor_driver")),
    )

    return LaunchDescription([
        launch_motor_arg,
        tracker_server_node,
        motor_driver_node,
    ])
