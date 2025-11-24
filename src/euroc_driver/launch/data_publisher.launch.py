from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('euroc_driver')
    
    # Config file paths
    default_config = os.path.join(pkg_dir, 'config', 'default_params.yaml')
    imu_config = os.path.join(pkg_dir, 'config', 'imu_config.yaml')
    cam0_config = os.path.join(pkg_dir, 'config', 'cam0_config.yaml')
    cam1_config = os.path.join(pkg_dir, 'config', 'cam1_config.yaml')
    # Declare launch arguments
    dataset_path_arg = DeclareLaunchArgument(
        'dataset_path',
        default_value='/workspace/data/MH_01_easy/mav0',
        description='Path to the EuRoC dataset directory'
    )
    
    playback_rate_arg = DeclareLaunchArgument(
        'playback_rate',
        default_value='1.0',
        description='Playback rate (1.0 = real-time, 2.0 = 2x speed)'
    )
    
    loop_playback_arg = DeclareLaunchArgument(
        'loop_playback',
        default_value='false',
        description='Whether to loop the dataset playback'
    )
    
    publish_images_arg = DeclareLaunchArgument(
        'publish_images',
        default_value='true',
        description='Whether to publish camera images'
    )
    
    # Create the data publisher node
    data_publisher_node = Node(
        package='euroc_driver',
        executable='data_publisher',
        name='euroc_data_publisher',
        output='screen',
        parameters=[
            default_config,
            imu_config,
            cam0_config,
            cam1_config,
            {
                'dataset_path': LaunchConfiguration('dataset_path'),
                'playback_rate': LaunchConfiguration('playback_rate'),
                'loop_playback': LaunchConfiguration('loop_playback'),
                'publish_images': LaunchConfiguration('publish_images'),
            }
        ],
        remappings=[
            # Remap topics if needed
            # ('imu/data', 'imu/data_raw'),
        ]
    )
    
    return LaunchDescription([
        dataset_path_arg,
        playback_rate_arg,
        loop_playback_arg,
        publish_images_arg,
        data_publisher_node
    ])
