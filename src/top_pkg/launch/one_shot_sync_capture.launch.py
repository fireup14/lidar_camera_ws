import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    sync_config = os.path.join(
        get_package_share_directory("top_pkg"),
        "config",
        "sync_capture.yaml",
    )

    sync_capture = Node(
        package="top_pkg",
        executable="sync_capture_node",
        name="sync_capture_node",
        output="screen",
        parameters=[sync_config],
    )

    return LaunchDescription([sync_capture])
