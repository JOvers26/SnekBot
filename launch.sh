#!/bin/bash

# Check virtual environment
if [ ! -d "venv" ]; then
    echo "❌ Python virtual environment not found!"
    exit 1
fi

# Activate Python virtual environment
source venv/bin/activate

# Source ROS 2 environment
source /opt/ros/jazzy/setup.bash

# Navigate to ROS 2 workspace
echo "🔧 Navigating to ROS 2 workspace..."
cd ~/SnekBot/ros2_ws || { echo "❌ Failed to navigate to ros2_ws directory."; exit 1; }

# Build workspace
echo "🚀 Running colcon build..."
colcon build --symlink-install

# Source setup file
echo "🔧 Sourcing the setup file..."
source install/setup.bash

# Launch micro_ros_agent
echo "🚀 Launching micro_ros_agent..."
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 &
MICRO_PID=$!

# Launch foxglove_bridge
echo "🌉 Launching foxglove_bridge..."
ros2 run foxglove_bridge foxglove_bridge &
BRIDGE_PID=$!

# Launch Python main
echo "🐍 Launching Raspimain.py..."
cd ~/SnekBot/RaspberryPi_Code || { echo "❌ Failed to navigate to RaspberryPi_Code directory."; exit 1; }
python3 Raspimain.py &
PYTHON_PID=$!

# Check webcam
if [ ! -e /dev/video0 ]; then
    echo "❌ Webcam not detected at /dev/video0!"
    exit 1
fi

# Launch MJPG-streamer
echo "📷 Launching MJPG-streamer..."
cd ~/SnekBot/mjpg-streamer-master/mjpg-streamer-experimental || { echo "❌ Failed to navigate to mjpg-streamer directory."; exit 1; }
./mjpg_streamer -i "./input_uvc.so -y -n -f 30 -r 640x480" -o "./output_http.so -w ./www" &
STREAMER_PID=$!

# Trap Ctrl+C and clean up
trap "echo '🛑 Caught interrupt, stopping...'; kill $MICRO_PID $BRIDGE_PID $PYTHON_PID $STREAMER_PID; exit 0" SIGINT

# Wait for background processes
wait $MICRO_PID $BRIDGE_PID $PYTHON_PID $STREAMER_PID
