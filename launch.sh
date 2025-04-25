#!/bin/bash

# Source ROS 2 environment
source /opt/ros/jazzy/setup.bash

# Navigate to the ROS 2 workspace
echo "🔧 Navigating to ROS 2 workspace..."
cd ~/SnekBot/ros2_ws || { echo "❌ Failed to navigate to ros2_ws directory."; exit 1; }

# Build the workspace
echo "🚀 Running colcon build..."
colcon build --symlink-install 

# Source the setup file after building
echo "🔧 Sourcing the setup file..."
source install/setup.bash

# Launching micro_ros_agent
echo "🚀 Launching micro_ros_agent..."
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 & 
MICRO_PID=$!

# Allow a short time to check if the agent started
sleep 2

# Check if the micro-ROS agent failed
if ! kill -0 $MICRO_PID 2>/dev/null; then
    echo "❌ micro_ros_agent failed to start."
    echo "🔧 Running build_micro_ros.sh and trying again..."

    # Kill any leftover process
    kill $MICRO_PID 2>/dev/null

    # Run the build script
    bash ~/ros2_ws/src/build_micro_ros.sh

    echo "🔁 Retrying micro_ros_agent..."

    # Retry launching micro-ROS agent
    ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 &
fi

# Wait for background processes
wait
