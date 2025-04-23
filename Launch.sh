#!/bin/bash

# Source ROS 2 environment
source /opt/ros/jazzy/setup.bash

echo "🚀 Launching foxglove_bridge and micro_ros_agent..."

# Try running both processes
ros2 run foxglove_bridge foxglove_bridge & 
FOX_PID=$!
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 & 
MICRO_PID=$!

# Allow a short time for both to initialize
sleep 2

# Check if either failed
if ! kill -0 $FOX_PID 2>/dev/null || ! kill -0 $MICRO_PID 2>/dev/null; then
    echo "❌ One or both processes failed to start."
    echo "🔧 Running build_micro_ros.sh and trying again..."

    # Kill any running leftover processes
    kill $FOX_PID $MICRO_PID 2>/dev/null

    # Run your build script
    bash ~/ros2_ws/src/build_micro_ros.sh

    echo "🔁 Retrying processes..."

    # Retry launching both
    ros2 run foxglove_bridge foxglove_bridge & 
    ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 &
fi

# Wait for background processes
wait
