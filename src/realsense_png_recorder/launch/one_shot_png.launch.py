from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory("realsense_png_recorder"),
        "config",
        "one_shot_png_recorder.yaml",
    )

    recorder = Node(
        package="realsense_png_recorder",
        executable="png_recorder_node",
        name="png_recorder_node",
        output="screen",
        parameters=[config_path],
    )

    return LaunchDescription([recorder])
