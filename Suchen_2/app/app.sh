#!/bin/bash
  
# Dieses Skript führt den Beispielcode für die Suchen 2 Labyrinthnavigation aus.
#
# This code is available under a GPL v3.0 license and comes without
# any explicit or implicit warranty.
#
# (C) Simon Schwaiger 2021 <schwaige@technikum-wien.at>

# Aktivierung der Python 3 und ROS Komponenten
source /opt/ros/melodic/setup.bash
source /catkin_ws/devel/setup.bash
source ~/myenv/bin/activate

# Generierung des Labyrinths
python3 generate_maze.py

# Deaktivierung des Python Virtual Environments, damit Gazebo richtig startet
deactivate

# Export des Pfads zum generierten Labyrinth
export GAZEBO_MODEL_PATH=/app #:$GAZEBO_MODEL_PATH

# Start der ROS Komponenten
roslaunch /app/sim.launch &

# Start der Gazebo Simulation
sleep 10
rosservice call /gazebo/unpause_physics

# Start der A Stern Navigation
source ~/myenv/bin/activate
python3 astar_navigation.py