#!/bin/bash

# Enable X11 forwarding for GUI applications
echo "Enabling X11 forwarding..."
xhost +local:docker

# Start the development container
echo "Starting First-Principles SLAM development container..."
docker compose up -d slam

# Attach to the container
echo "Attaching to container..."
docker compose exec slam bash
