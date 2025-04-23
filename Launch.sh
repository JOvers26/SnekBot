#!/bin/bash

# Source ROS 2 environment
source /opt/ros/jazzy/setup.bash

echo "🚀 Launching micro_ros_agent..."

# Attempt to start micro_ros_agent
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 &
AGENT_PID=$!

# Allow time for the agent to initialize
sleep 2

# Verify the process is running
if ! kill -0 $AGENT_PID 2>/dev/null; then
    echo "❌ micro_ros_agent failed to start."
    echo "🔧 Running build_micro_ros.sh to resolve issues..."

    # Terminate any lingering process (if somehow partially running)
    kill $AGENT_PID 2>/dev/null

    # Execute the build script to rebuild the environment
    bash ~/ros2_ws/src/build_micro_ros.sh

    echo "🔁 Retrying micro_ros_agent launch..."

    # Relaunch the agent
    ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 &
fi

# Maintain script until background process ends
wait
