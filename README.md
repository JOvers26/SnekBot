# SnekBot

## Six-Axis Robotic Arm and Control System for ROAR Lunar Surface Operations
This project is a work in progress and setup is subject to change.

## Project Summary
SnekBot is a six-axis robotic arm designed for the QUT ROAR team to use in the Australian Rover Challenge (ARCh)



## Installation & Setup
### Initial Steps
Ensure Ubuntu 24.04 server is flashed on the Raspberry Pi 5 and configured for remote access via SSH.

## **1. Set Locale**
Ensure your system uses **UTF-8** encoding:

```sh
locale  # Check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # Verify settings
```

## **2. Enable Required Repositories**

```sh
sudo apt install software-properties-common
sudo add-apt-repository universe
```

## **3. Add ROS 2 GPG Key**

```sh
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

## **4. Add ROS 2 Repository**

```sh
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

## **5. Install Development Tools (Optional)**

```sh
sudo apt update && sudo apt install ros-dev-tools
```

## **6. Update Repository Caches**

```sh
sudo apt update
```

## **7. Upgrade System**

```sh
sudo apt upgrade
```

## **8. Install ROS 2 Jazzy Base**

```sh
sudo apt install ros-jazzy-ros-base
```

## **9. Setup Environment**
Ensure ROS 2 is sourced correctly:

```sh
# Replace ".bash" with your shell if you're not using Bash
# Possible values: setup.bash, setup.sh, setup.zsh
source /opt/ros/jazzy/setup.bash
```


