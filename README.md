# SnekBot

## Six-Axis Robotic Arm and Control System for ROAR Lunar Surface Operations

This project is a work in progress and setup is subject to change.

## Project Summary
SnekBot is a six-axis robotic arm designed for the QUT ROAR team to use in the Australian Rover Challenge (ARCh)

Key Features:
- **Modular Design:** 3D-printed and aluminum frame for durability and customization.
- **ESP32 Motor Control:** Interprets ROS2 movement commands and sends precise motor signals.
- **Raspberry Pi 5 Compute Module:** Handles ROS2 for motion planning and high-level control.
- **ROS2 Integration:** Allows seamless component integration and future applicaitons

## Project Goal
SnekBot aims to provide a robust and adaptable robotic arm platform for simulated lunar applications

## Installation & Setup
### Initial Steps
Clone SnekBot repository:

```bash
git clone https://github.com/JOvers26/SnekBot.git ~/SnekBot
```

#### Set locale:
Install ROS 2 Jazzy (Ubuntu Noble 24.04)
```bash
cd ~/SnekBot
chmod +x install.sh
chmod +x uninstall.sh
./install.sh
```

#### Building the ROS2 Workspace:
```bash
cd ~/SnekBot
colcon build --symlink-install
source install/setup.bash
```

#### Install and activate virtual environment:
```bash
sudo apt install -y python3.12-venv
python3 -m venv ~/SnekBot/venv
source ~/SnekBot/venv/bin/activate
```


#### Installing robotics toolbox python
```bash
pip install --upgrade pip
pip install setuptools
pip install roboticstoolbox-python
pip install "numpy<2"
```

#### Change USB permissions
```bash
sudo usermod -aG dialout $USER
newgrp dialout
```


