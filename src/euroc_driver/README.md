# EuRoC Data Publisher

A ROS2 node for publishing EuRoC dataset data including IMU, camera images, and ground truth pose information.

## Features

- **IMU Data Publishing**: Publishes angular velocity and linear acceleration data
- **Camera Image Publishing**: Publishes stereo camera images (cam0 and cam1)
- **Ground Truth Pose Publishing**: Publishes position, orientation, and velocity data
- **Configurable Playback Rate**: Control the speed of data playback
- **Proper Timing**: Maintains original dataset timing relationships
- **Modular Design**: Clean separation of concerns with dedicated parsers and readers

## Topics Published

- `/imu/data` (sensor_msgs/Imu): IMU measurements
- `/cam0/image_raw` (sensor_msgs/Image): Left camera images
- `/cam1/image_raw` (sensor_msgs/Image): Right camera images  
- `/pose` (geometry_msgs/PoseStamped): Ground truth pose
- `/velocity` (geometry_msgs/TwistStamped): Ground truth velocity

## Parameters

- `dataset_path` (string): Path to the mav0 directory of the EuRoC dataset
- `playback_rate` (double): Playback speed multiplier (default: 1.0)
- `loop_playback` (bool): Whether to loop the dataset (default: false)
- `publish_images` (bool): Whether to publish camera images (default: true)

## Usage

### Building

```bash
cd /workspace
colcon build --packages-select euroc_driver
source install/setup.bash
```

### Running with Launch File

```bash
# Basic usage with default parameters
ros2 launch euroc_driver data_publisher.launch.py

# With custom parameters
ros2 launch euroc_driver data_publisher.launch.py \
    dataset_path:=/path/to/your/dataset/mav0 \
    playback_rate:=0.5 \
    publish_images:=false
```

### Running Directly

```bash
# Run with default parameters
ros2 run euroc_driver data_publisher

# Run with custom parameters
ros2 run euroc_driver data_publisher \
    --ros-args \
    -p dataset_path:=/path/to/your/dataset/mav0 \
    -p playback_rate:=2.0 \
    -p loop_playback:=true
```

### Using Configuration File

```bash
ros2 run euroc_driver data_publisher \
    --ros-args \
    --params-file src/euroc_driver/config/default_params.yaml
```

## Dataset Structure

The node expects the following EuRoC dataset structure:

```
mav0/
├── imu0/
│   ├── data.csv
│   └── sensor.yaml
├── cam0/
│   ├── data.csv
│   ├── sensor.yaml
│   └── data/
│       ├── *.png
├── cam1/
│   ├── data.csv
│   ├── sensor.yaml
│   └── data/
│       ├── *.png
└── state_groundtruth_estimate0/
    ├── data.csv
    └── sensor.yaml
```

## Architecture

The code follows good software engineering practices:

### Components

1. **CSVReader**: Generic CSV file reader utility
2. **DataParsers**: Specialized parsers for different data types (IMU, Camera, Pose)
3. **DataPublisher**: Main ROS2 node that orchestrates everything

### Design Principles

- **Single Responsibility**: Each class has a clear, focused purpose
- **Dependency Injection**: Parsers are injected into the main publisher
- **Error Handling**: Robust error handling for file operations and data parsing
- **Configurability**: Extensive parameter support for different use cases
- **Resource Management**: Proper RAII and smart pointer usage

## Monitoring Data

```bash
# List all topics
ros2 topic list

# Monitor IMU data
ros2 topic echo /imu/data

# Monitor pose data
ros2 topic echo /pose

# Check message rates
ros2 topic hz /imu/data
```

## Troubleshooting

### Common Issues

1. **File not found errors**: Ensure the dataset_path parameter points to the correct mav0 directory
2. **No images published**: Check that image files exist in cam0/data and cam1/data directories
3. **Timing issues**: Adjust the playback_rate parameter if data is too fast/slow

### Debug Information

The node provides detailed logging. Run with debug level:

```bash
ros2 run euroc_driver data_publisher --ros-args --log-level debug
```
