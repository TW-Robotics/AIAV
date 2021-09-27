#!/bin/bash

# Dieses Skript erstellt die Umgebung zur Ausführung des Policy Iteration Pick and Place Usecases
#
# This code is available under a GPL v3.0 license and comes without
# any explicit or implicit warranty.
#
# (C) Simon Schwaiger 2021 <schwaige@technikum-wien.at>

# Download des ROS ML Containers
git clone https://github.com/SimonSchwaiger/ros-ml-container

# Kopieren der Applikation
rm -rf ros-ml-container/app
cp app ros-ml-container
cp requirements.txt ros-ml-container/requirements.txt

# Download der benötigten Software Pakete
cd ros-ml-container && mkdir src
cd ros-ml-container/src && git clone https://github.com/TW-Robotics/RL-with-3DOF-Robots
cd ros-ml-container/src && git clone https://github.com/HebiRobotics/hebi_description

