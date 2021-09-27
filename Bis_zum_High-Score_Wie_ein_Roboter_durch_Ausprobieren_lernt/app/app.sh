#!/bin/bash

# Dieses Skript führt die Komponenten für den Pick and Place Usecase in Simulation aus.
#
# This code is available under a GPL v3.0 license and comes without
# any explicit or implicit warranty.
#
# (C) Simon Schwaiger 2021 <schwaige@technikum-wien.at>


# Initialisierung der ROS und Pyhton Komponenten
source ~/myenv/bin/activate
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash

# Installation des Gym Environments
pip3 install -e /catkin_ws/src/RL-with-3DOF-Robots/fhtw3dof/gym-fhtw3dof

# Start der ROS Simulation
roslaunch saimon SAImon.launch coll_map:=usecase.yaml run_on_real_robot:=false &

# Start der Simulierten Kamera
python interactive_marker.py &

# Start der Pick and Place Applikation
python pick_and_place.py

