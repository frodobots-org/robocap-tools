#!/bin/bash
set -e

echo "=== Calibration API Server ==="

# Source ROS environments
echo "Sourcing ROS environments..."
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash
source /catkin_ws_imu/devel/setup.bash

# Set Kalibr environment variable
export KALIBR_MANUAL_FOCAL_LENGTH_INIT=1

# Install Python dependencies
echo "Installing Python dependencies..."
pip install -q -r /robocap-tools/api/requirements.txt

# Start the API server
echo "Starting API server on port 8080..."
cd /robocap-tools/api
exec python3 app.py
