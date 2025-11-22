#!/bin/bash

# Build the Docker image
echo "Building Docker image for First-Principles SLAM..."
docker build -t fp-slam:humble .

echo "Docker image built successfully!"
echo "To run the container, use:"
echo "  docker compose up slam"
echo ""
echo "To run with GUI support, make sure X11 forwarding is enabled:"
echo "  xhost +local:docker"
echo ""

