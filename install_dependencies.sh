#!/bin/bash

sudo apt-get update && sudo apt-get upgrade
sudo apt-get install ros-melodic-uol-cmp9767m-base ros-melodic-desktop
source /opt/ros/melodic/setup.bash

sudo apt install python-pip
pip install numpy
pip install -U scikit-learn
sudo apt-get install ros-melodic-opencv-apps
sudo apt-get install ros-melodic-topological-utils
sudo apt-get install ros-melodic-topological-navigation
sudo apt-get install ros-melodic-topological-navigation-msgs
sudo apt-get install ros-melodic-strands-navigation
sudo apt-get install ros-melodic-image-geometry
exit