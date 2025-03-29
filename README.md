# SnekBot

![SnekBot Arm](Images/SnekBot.png)

## Six-Axis Robotic Arm and Control System for ROAR Lunar Surface Operations
This project is a work in progress and setup is subject to change.

## Project Summary
SnekBot is a six-axis robotic arm designed for the QUT ROAR team to use in the Australian Rover Challenge (ARCh)



## Installation & Setup
### Initial Steps
Ensure Ubuntu 24.04 is flashed on the Raspberry Pi 5 and configured for remote access via SSH.

## ROS2 Jazzy
### **1. Set Locale**
Ensure your system uses **UTF-8** encoding:
```sh
locale  # Check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # Verify settings
```

### **2. Enable Required Repositories**
```sh
sudo apt install software-properties-common
sudo add-apt-repository universe
```

### **3. Add ROS 2 GPG Key**
```sh
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

### **4. Add ROS 2 Repository**
```sh
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### **5. Install Development Tools (Optional)**
```sh
sudo apt update && sudo apt install ros-dev-tools
```

### **6. Update Repository Caches**
```sh
sudo apt update
```

### **7. Upgrade System**
```sh
sudo apt upgrade
```

### **8. Install ROS 2 Jazzy Base**
```sh
sudo apt install ros-jazzy-ros-base
```

### **9. Setup Environment**
Ensure ROS 2 is sourced correctly:
```sh
# Replace ".bash" with your shell if you're not using Bash
# Possible values: setup.bash, setup.sh, setup.zsh
source /opt/ros/jazzy/setup.bash
```
For more details, visit the [official ROS 2 documentation](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).

## Setting up SnekBot Project
### **1. Clone the repository**
```sh
git clone https://github.com/JOvers26/SnekBot
cd ~/Snekbot
```

### **2. Install dependencies**
```sh
cd /ros2_ws
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### **3. Install FOXGLOVE Monitoring (Optional)**
```sh
source /opt/ros/jazzy/setup.bash
sudo apt install ros-$ROS_DISTRO-foxglove-bridge
```

### **4. Build the ROS2 Workspace:**
```sh
colcon build --symlink-install
source install/setup.bash
```

## Setting up ESP-IDF
### **1. Install Prerequisites**
```sh
sudo apt-get install git wget flex bison gperf python3 python3-pip python3-venv cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0
```

### **2. Get ESP-IDF**
```sh
mkdir -p ~/esp
cd ~/esp
git clone -b v5.4.1 --recursive https://github.com/espressif/esp-idf.git
```

### **3. Set up the Tools**
```sh
cd ~/esp/esp-idf
./install.sh esp32s3
```

### **4. Set up the Environment Variables**
```sh
. $HOME/esp/esp-idf/export.sh
```

### **4. micro-ROS component for ESP-IDF**
```sh
pip3 install catkin_pkg lark-parser colcon-common-extensions empy==3.3.4
```




pip3 install catkin_pkg lark-parser colcon-common-extensions empy==3.3.4

# in micro_ros settings set WIFI and Password ssh
cd ~/SnekBot/esp-idf-master/components/micro_ros_espidf_components-jazzy/examples/int32_publisher
idf.py set-target esp32s3
idf.py menuconfig




ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0



idf.py set-target esp32
idf.py menuconfig
idf.py -p /dev/ttyACM0 flash
idf.py build
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
sudo apt install python3-venv
mkdir snekbot_ws


### **5. Install and activate virtual ennvirnoment**
```sh
python3 -m venv venv
source venv/bin/activate
```

### **5. Install Robotics Toolbox**
```sh
pip3 install pygame
pip3 install setuptools
pip3 install roboticstoolbox-python
pip3 install "numpy<2"
```
