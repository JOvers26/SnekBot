#!/bin/bash

sudo apt remove ~nros-jazzy-* && sudo apt autoremove'
sudo rm /etc/apt/sources.list.d/ros2.list
sudo apt -y update
sudo apt iy autoremove
# Consider upgrading for packages previously shadowed.
sudo apt -y upgrade
cd $HOME
rm -r SnekBot/