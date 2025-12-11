#!/bin/bash
# Execute a command inside the docker container with full environment setup

# Check if container is running
if ! docker ps | grep -q fp_slam_container; then
    echo "Error: fp_slam_container is not running."
    exit 1
fi

# Determine if we are in an interactive terminal
DOCKER_FLAGS=""
if [ -t 1 ]; then
    DOCKER_FLAGS="-it"
fi

# Execute command
docker exec $DOCKER_FLAGS fp_slam_container bash -c 'source /opt/ros/humble/setup.bash; if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi; exec "$@"' -- "$@"
