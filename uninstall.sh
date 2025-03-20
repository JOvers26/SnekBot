#!/bin/bash

sudo apt remove ~nros-jazzy-* && sudo apt autoremove
sudo rm /etc/apt/sources.list.d/ros2.list
sudo apt -y update
sudo apt iy autoremove
sudo apt -y upgrade
cd $HOME
rm -r SnekBot/