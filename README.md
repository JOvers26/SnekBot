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
cd ~/SnekBot
chmod +x install.sh
```

#### Set locale:
Install ROS 2 Jazzy (Ubuntu Noble 24.04)
```bash
./install.sh
```


