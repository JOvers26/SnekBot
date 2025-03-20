#!/bin/bash

# Exit immediately if a command fails
echo "📌 Install will exit immediately if a command fails..."
set -e  

echo "🚀 Starting SnekBot installation..."

# Update package list
sudo apt update

echo "📌 Setting up locale..."
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

echo "📌 Enabling required repositories..."
sudo apt install -y software-properties-common
sudo add-apt-repository -y universe

echo "📌 Adding ROS 2 GPG key..."
sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "📌 Adding ROS 2 repository..."
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo "📌 Installing development tools"
sudo apt update && sudo apt install -y ros-dev-tools

echo "📌 Installing ROS 2 Jazzy Base..."
sudo apt update
sudo apt -y upgrade
sudo apt install -y ros-jazzy-ros-base

echo "📌 Sourcing ROS2 installation"
source /opt/ros/jazzy/setup.bash

# Install additional dependencies required for building ROS 2 messages
echo "📌 Installing required dependencies for ROS 2 message generation..."
sudo apt install -y ros-jazzy-rosidl-typesupport-c
sudo apt install -y ros-jazzy-rosidl-default-generators

echo "📌 Installing Micro-ROS Packages..."
mkdir ~/SnekBot/src
cd ~/SnekBot/src
git clone https://github.com/micro-ROS/micro-ROS-Agent -b jazzy
git clone https://github.com/micro-ROS/micro_ros_msgs -b jazzy

echo "📌 Installing FoxGlove Monitoring..."
sudo apt install -y ros-$ROS_DISTRO-foxglove-bridge

echo "📌 Installing Dependencies"
cd ~/SnekBot

# Check if the rosdep sources file exists, and remove it if found
if [ -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  echo "⚠️ Existing rosdep sources list found. Removing..."
  sudo rm /etc/ros/rosdep/sources.list.d/20-default.list
fi

# Reinitialize rosdep and update sources
sudo rosdep init
rosdep update

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

echo "📌 Building the ROS2 Workspace"
cd ~/SnekBot
colcon build --symlink-install
source install/setup.bash

echo "📌 Rebuild ROS2 Workspace"
cd ~/SnekBot
colcon build --symlink-install
source install/setup.bash

echo "📌 Install and activate virtual environment"
sudo apt install -y python3.12-venv
python3 -m venv ~/SnekBot/venv
source ~/SnekBot/venv/bin/activate

echo "📌 Installing robotics toolbox python"
pip install --upgrade pip
pip install setuptools
pip install roboticstoolbox-python

echo "📌 Fixing numpy version dependancies"
pip install "numpy<2"

echo "📌 Change USB permissions"
sudo usermod -aG dialout $USER
newgrp dialout