#!/bin/bash

# Remove all ROS 2 Jazzy packages and automatically confirm
echo "📌 Removing ROS 2 Jazzy packages..."
sudo apt -y remove ros-jazzy-* && sudo apt -y autoremove

# Remove the ROS 2 repository list file from the sources directory
echo "📌 Removing ROS 2 source list..."
sudo rm -f /etc/apt/sources.list.d/ros2.list

# Update package list after removing the repository
echo "📌 Updating package list..."
sudo apt -y update

# Run autoremove to clean up any leftover dependencies
echo "📌 Running autoremove to clean up unused packages..."
sudo apt -y autoremove

# Upgrade any remaining packages to their latest versions
echo "📌 Upgrading packages..."
sudo apt -y upgrade

# Navigate to the home directory
echo "📌 Navigating to the home directory..."
cd $HOME

# Remove the 'SnekBot' directory
echo "📌 Removing the 'SnekBot' directory..."
rm -rf SnekBot/

# Done
echo "📌 Cleanup and removal complete!"
