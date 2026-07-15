#!/usr/bin/env bash
set -Eeuo pipefail

if ! command -v sudo >/dev/null 2>&1; then
    echo "sudo is required to install RealSense packages." >&2
    exit 1
fi

sudo apt update
sudo apt install -y \
    ros-humble-realsense2-camera \
    ros-humble-realsense2-description \
    librealsense2-utils \
    librealsense2-dev

echo "RealSense packages installed."
echo "Verify connected cameras with:"
echo "  rs-enumerate-devices"
echo "Verify ROS launch with:"
echo "  ros2 launch realsense2_camera rs_launch.py"
