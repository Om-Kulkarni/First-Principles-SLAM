# First Principles SLAM - Docker Setup

This project now includes Docker support for ROS2 Humble.

## Prerequisites

- Docker
- Docker Compose
- X11 forwarding support (for GUI applications like RViz)

## Quick Start

### 1. Build the Docker Image

```bash
./docker/build.sh
```

Or manually:

```bash
docker build -t fp-slam:humble .
```

### 2. Run the Development Container

```bash
./docker/run.sh
```

This will:

- Enable X11 forwarding for GUI applications
- Start the development container
- Attach to an interactive bash session

### 3. Build the ROS2 Workspace (inside container)

```bash
colcon build
source install/setup.bash
```

## Docker Services

The `docker-compose.yml` defines several services:

### Development Container

```bash
docker compose up slam
```

Interactive container for development and testing.

## Manual Docker Commands

### Build and Run Interactively

```bash
docker build -t fp-slam:humble .
docker run -it --rm \
  --network host \
  -v $(pwd):/workspace \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  fp-slam:humble bash
```

## GUI Applications

For GUI applications like RViz to work, you need to enable X11 forwarding:

```bash
xhost +local:docker
```

## Troubleshooting

### X11 Issues

If you encounter display issues:

```bash
echo $DISPLAY
xhost +local:docker
```

### Permission Issues

If you have permission issues with mounted volumes:

```bash
sudo chown -R $USER:$USER .
```

### Container Cleanup

To clean up containers and images:

```bash
docker compose down
docker system prune -a
```
