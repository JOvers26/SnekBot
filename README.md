# SnekBot

## Six-Axis Robotic Arm for Intelligent Automation and Research

This project is a work in progress and setup is subject to change.

## Project Summary
SnekBot is a six-axis robotic arm designed for research and automation tasks. It is built with ROS2 integration, allowing precise motion planning, inverse kinematics, and real-time control. The arm is intended for applications in pick-and-place operations, object manipulation, and autonomous robotic tasks.

Key Features:
- **Modular Design:** 3D-printed and aluminum frame for durability and customization.
- **ESP32 Motor Control:** Interprets ROS2 movement commands and sends precise motor signals.
- **Raspberry Pi 5 Compute Module:** Handles ROS2 for motion planning and high-level control.
- **Custom Motor Driver PCB:** Integrated power protection features and improved motor control.
- **Sensor Suite:** Includes an IMU and optional vision system for advanced feedback.
- **ROS2 Integration:** Allows autonomous control, path planning, and real-time adjustments.

## Project Goal
SnekBot aims to provide a robust and adaptable robotic arm platform for research and automation. It integrates ROS2-based motion control, customizable hardware, and modular components for flexibility in various applications.

## Table of Contents
- Project Structure
- Installation & Setup
- Initial Steps
- Installing ROS2 Jazzy
- Setting up SnekBot Project
- Motor Driver Microcontroller Flashing
- Running SnekBot in ROS2
- Tips and Tricks
- General ROS2 Practices
- License
- Troubleshooting

## Project Structure
```
ros2_ws/
│── build/          (Generated after colcon build)
│── install/        (Generated after colcon build)
│── src/            (Generated after colcon build)
│── log/            (Generated after colcon build)
├── scripts/        # Standalone scripts for SnekBot
│   ├── control.py  # Python script for arm control
├── ESP32_Code/     # Code for the ESP32 motor driver
│   ├── firmware/   # ESP32 firmware files
│── README.md       # This file
│── .gitignore      # Files to ignore when committing to GitHub
```

## Features Being Worked On
- Remote control via keyboard & joystick
- Computer vision-based object tracking
- IMU-based stabilization
- Inverse kinematics for precise motion control
- Web interface for remote operation
- Advanced path planning and obstacle avoidance

## Installation & Setup
### Initial Steps
Ensure **Ubuntu 24.04** is installed on the Raspberry Pi 5 and configured for remote access via SSH.

### Installing ROS2 Jazzy
Follow the official ROS2 Jazzy installation guide or use the steps below.

#### Set locale:
```bash
locale  # check for UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale  # verify settings
```
#### Enable required repositories:
```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```
#### Add ROS2 GPG key:
```bash
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```
#### Add ROS2 repository to sources list:
```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```
#### Install ROS2 Jazzy Base:
```bash
sudo apt update && sudo apt install ros-jazzy-ros-base
```
#### Setup environment:
```bash
source /opt/ros/jazzy/setup.bash
```

### Setting Up SnekBot Project
#### Clone the repository:
```bash
git clone https://github.com/YourGitHubUsername/SnekBot.git ~/ros2_ws
cd ~/ros2_ws
```
#### Install required packages:
```bash
mkdir ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/micro-ROS/micro-ROS-Agent -b jazzy
git clone https://github.com/micro-ROS/micro_ros_msgs -b jazzy
```
#### Install dependencies:
```bash
cd ~/ros2_ws
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```
#### Build the ROS2 workspace:
```bash
colcon build --symlink-install
source install/setup.bash
```

## Motor Driver Microcontroller Flashing
Flash ESP32 firmware using Arduino IDE or PlatformIO. Ensure **micro-ROS Agent is not running** during flashing.

## Running SnekBot in ROS2
1. Connect the motor driver microcontroller to the Raspberry Pi.
2. Start the micro-ROS agent:
   ```bash
   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
   ```

## Tips and Tricks
### Working with GitHub through Terminal on the Pi5
#### Understanding `git pull` vs `git fetch`
- `git fetch` checks for updates without modifying local files.
- `git pull` updates your local branch with the latest changes.

#### Common Git Commands
```bash
git fetch origin  # Retrieve changes
git pull origin main  # Fetch and merge updates
git checkout -b feature-branch  # Create a new branch
git checkout main  # Switch back to main
git merge feature-branch  # Merge changes
git stash  # Save uncommitted changes
git log --oneline --graph --all --decorate  # View commit history
```

## General ROS2 Practices
### Sourcing the Environment
```bash
source install/setup.bash
```
### Building the Workspace
```bash
colcon build --symlink-install
source install/setup.bash
```
### Checking Available ROS2 Nodes and Topics
```bash
ros2 node list
ros2 topic list
ros2 topic echo /your_topic_name
```
### Running ROS2 Packages
```bash
ros2 launch package_name launch_file.py
ros2 run package_name node_name
python filename.py
```
### Manually Sending ROS2 Messages
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}" --once
```

## Troubleshooting
TBD

## License
MIT License (or your preferred license)

---

Let me know if you'd like any modifications! 🚀

