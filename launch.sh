#!/bin/bash

# Activate Python virtual environment
source venv/bin/activate

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

# Launching foxglove_bridge
echo "🌉 Launching foxglove_bridge..."
ros2 run foxglove_bridge foxglove_bridge &
BRIDGE_PID=$!

# Launching Raspimain.py
echo "🐍 Launching Raspimain.py..."
cd ~/SnekBot/RaspberryPi_Code || { echo "❌ Failed to navigate to RaspberryPi_Code directory."; exit 1; }
python3 Raspimain.py &
PYTHON_PID=$!

# Allow time for all processes to initialize
sleep 2

# Wait for all background processes
wait $MICRO_PID $BRIDGE_PID $PYTHON_PID
