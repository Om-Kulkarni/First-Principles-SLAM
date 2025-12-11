from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_vio = get_package_share_directory("vio_frontend")
    pkg_euroc = get_package_share_directory("euroc_driver")

    # EuRoC Configs for Parameters
    cam0_config = os.path.join(pkg_euroc, "config", "cam0_config.yaml")

    # Launch EuRoC Driver
    euroc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_euroc, "launch", "data_publisher.launch.py")
        ),
        launch_arguments={
            "dataset_path": "/workspace/data/MH_01_easy/mav0",
            "publish_images": "true",
            "playback_rate": "1.0",
        }.items(),
    )

    tracker_config = os.path.join(pkg_vio, "config", "tracker.yaml")

    # Launch Feature Tracker
    tracker_node = Node(
        package="vio_frontend",
        executable="feature_tracker_node",
        name="feature_tracker",
        output="screen",
        parameters=[tracker_config],
    )

    return LaunchDescription([euroc_launch, tracker_node])
