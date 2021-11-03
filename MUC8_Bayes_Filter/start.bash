#!/bin/bash
#set -v -x
echo -e "\e[33mStarting Project Kalman Filter\e[0m"
xterm -e "killall -9 gzserver" &
sleep 1
xterm -e "killall -9 gzclient" &
sleep 1

echo "Starting Roscore..."
xterm -e "roscore" &
sleep 10
echo "Setting up Gazebo..."
xterm -e "roslaunch soar_pr5 mir_maze_world.launch" &
sleep 20
xterm -e "rosservice call /gazebo/unpause_physics"
sleep 5
echo "Starting MiR amcl..."
xterm -e "roslaunch mir_navigation amcl.launch initial_pose_x:=5.0 initial_pose_y:=5.0 map:=$(rospack find soar_pr5)/maps/map.yaml" &
sleep 5
echo "Starting MiR planner..."
xterm -e "roslaunch mir_navigation start_planner.launch     map_file:=$(rospack find soar_pr5)/maps/map.yaml" &
sleep 5
echo "Starting Laserscan Node..."
xterm -hold -e "rosrun soar_pr5 soar_pr5_laser" &
sleep 5
echo "Starting Task Node..."
xterm -hold -e "rosrun soar_pr5 soar_pr5_task" &
sleep 5
echo "Starting Kalman filter Node..."
xterm -hold -e "rosrun soar_pr5 soar_pr5_node" &
sleep 5
echo "Starting RViz..."
xterm -e "rviz -d $(rospack find soar_pr5)/rviz/navigation.rviz" &
sleep 5
echo "Starting Evaluation Node..."
xterm -hold -e "rosrun soar_pr5 soar_pr5_evaluation" &
sleep 5
echo "Starting AmclPose tf..."
xterm -e "rosrun soar_pr5 soar_pr5_amclPose" &
sleep 5
echo "Starting rqt..."
xterm -e "rqt" &
sleep 5
echo -e "\e[32mFinished\e[0m"
echo -e "\e[32mTo start the program: Set a near 2D Nav Goal in RViz\e[0m"
wait
