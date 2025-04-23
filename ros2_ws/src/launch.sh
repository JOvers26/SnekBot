source /opt/ros/jazzy/setup.bash

colcon build --symlink-install
source install/setup.bash


ros2 run foxglove_bridge foxglove_bridge
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888