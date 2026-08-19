import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def load_bringup_config():
    config_path = os.path.join(
        get_package_share_directory("top_pkg"),
        "config",
        "bringup.yaml",
    )
    with open(config_path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)["bringup"]


def generate_launch_description():

    config = load_bringup_config()
    livox_cfg = config["livox"]
    realsense_cfg = config["realsense"]
    livox_config_path = os.path.join(
        get_package_share_directory("livox_ros_driver2"),
        "config",
        livox_cfg["config_file"],
    )

    enable_lidar_arg = DeclareLaunchArgument(
        "enable_lidar",
        default_value=str(config["enable_lidar"]).lower(),
    )
    enable_camera_arg = DeclareLaunchArgument(
        "enable_camera",
        default_value=str(config["enable_camera"]).lower(),
    )
    enable_rviz_arg = DeclareLaunchArgument(
        "enable_rviz",
        default_value=str(config["enable_rviz"]).lower(),
    )

    livox_driver = Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="livox_lidar_publisher",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_lidar")),
        parameters=[
            {"xfer_format": 0},
            {"multi_topic": 0},
            {"data_src": 0},
            {"publish_freq": livox_cfg["publish_freq"]},
            {"output_data_type": 0},
            {"frame_id": livox_cfg["frame_id"]},
            {"user_config_path": livox_config_path},
            {"cmdline_input_bd_code": livox_cfg["cmdline_input_bd_code"]},
            {"lvx_file_path": livox_cfg["lvx_file_path"]},
        ],
    )

    # Start the camera node directly. Including rs_launch.py would expose this
    # launch file's enable_* switches to the driver's unsupported-param checker.
    realsense_driver = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        namespace=realsense_cfg["camera_namespace"],
        name=realsense_cfg["camera_name"],
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_camera")),
        parameters=[{
            "camera_name": realsense_cfg["camera_name"],
            "camera_namespace": realsense_cfg["camera_namespace"],
            "device_type": realsense_cfg["device_type"],
            # Keep parameter types native.  Quoted booleans are passed as
            # strings and rejected by realsense2_camera.
            "enable_color": True,
            # This bringup only needs RGB images and their CameraInfo.
            "enable_depth": False,
            "pointcloud.enable": False,
            "align_depth.enable": False,
            "enable_infra": False,
            "enable_infra1": False,
            "enable_infra2": False,
            # D405 exposes its color stream through the depth module.
            "depth_module.color_profile": realsense_cfg["color_profile"],
        }],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="calibration_rviz",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_rviz")),
        arguments=["-d", os.path.join(
            get_package_share_directory("top_pkg"), "config", "bringup.rviz")],
    )

    return LaunchDescription([
        enable_lidar_arg,
        enable_camera_arg,
        enable_rviz_arg,
        livox_driver,
        realsense_driver,
        rviz,
    ])
