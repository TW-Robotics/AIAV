#!/usr/bin/env python3

# Dieses Skript implementiert die Ablaufsteuerung für den AIAV Policy Iteration Pick and Place Usecase.
#
# This code is available under a GPL v3.0 license and comes without
# any explicit or implicit warranty.
#
# (C) Simon Schwaiger 2021 <schwaige@technikum-wien.at>

import numpy as np
import gym
import gym_fhtw3dof
import time
import rospy
import tf
import math

from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Bool

import fhtw3dof_usecase_tools
import inaccuracy_compensation

# Controls, whether or not the usecase is performed on the real robot or just in simulation.
real = False

# Set place state and home state
place_state = 34721
home_state = 29659

# Set kinematics parameters
goalstate_distance = 0.04
grasp_height = -0.01
goalstate_height = 0.04

# Set 
save_model = True
load_pretrained = True


"""Control script for running the Usecase of the SAImon Robot."""

def find_nearest_state(point, max_distance, env):
    """ Returns the goalstate with the smallest distance. Only states within max_distance are considered. """
    if (not isinstance(point, Point)): return -1
    states = env.find_near_states(point, max_distance)
    if len(states) == 1: return states[0]
    #max_distance = max_distance
    goalstate = -1
    for s in states:
        distance =  round(math.sqrt(math.pow(point.x - env.TCP_Positions[s].x, 2) + 
                                    math.pow(point.y - env.TCP_Positions[s].y, 2)), 6)
        try:
            if distance < max_distance:
                max_distance = distance
                goalstate = s
        except ValueError:
            continue
    return goalstate

def get_object_position():
    """ Gets the current object position from the tf-color-transformation transformpublisher and converts it to a valid robot state. """
    pt = PointStamped()
    pt.header.frame_id = "object_tf"
    pt.header.stamp = rospy.Time()
    pt.point.x = 0
    pt.point.y = 0
    pt.point.z = 0
    try:
        pt = tflistener.transformPoint("world", pt)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print("No Transform available")
        return -1
    print(pt)
    return pt

def compute_goal(env, goalstate_distance):
    """ Determines RL goalstate of the robot as well as the jointstates to precisely reach the goalstate. """
    pt = get_object_position()
    if pt == -1: return -1, 0, 0, 0
    try:
        j3, j2, j1 = inaccuracy_compensation.getJointstate(pt.point.x, pt.point.y, grasp_height)
    except ValueError:
        print("Could not solve inverse kinematics")
        return -1, 0, 0, 0
    pt.point.z = goalstate_height
    goalstate = find_nearest_state(pt.point, goalstate_distance, env)
    return goalstate, j3, j2, j1

def goto_jointstate(j3, j2, j1, env):
    """ Moves the robot joints to j3, j2 and j1. """
    env.js.position[0] = j1
    env.js.position[1] = j2
    env.js.position[2] = j3
    env.render()
    if real: hebi_pub.add_waypoint()

def apply_policy(policy, target_state, env):
    """ Performs a learned policy on the simulated and real robots. """
    obs = env.s
    step_idx = 0
    while True:
        action = int(policy[obs])
        obs, _, _, _ = env.step(action)
        step_idx += 1
        env.render_state(obs)
        time.sleep(0.08)
        if real and (int(step_idx%2)==1): hebi_pub.add_waypoint()
        #if real: hebi_pub.add_waypoint()
        if obs == target_state: 
            return 1
        if step_idx >= 100: return -1
    return -1

def setup_environment():
    """ Sets up the FHTW3DOF gym environment for usage with the SAImon robot for the usecase. """
    env = gym.make('fhtw3dof-v0', Stop_Early=False, Constrain_Workspace=True,
                    GoalX=0.179, GoalY=0.0026, GoalZ=0.335,
                    J1Limit=33, J2Limit=35, J3Limit=37, joint_res = 0.08,
                    J1Scale=1, J2Scale=-1, J3Scale=-1, J3Offset=0.5,
                    Debug=True, Publish_Frequency=500)
    assert (env.reachable)
    env.Publish_Frequency = 20
    return env

###############################

rospy.init_node("aiav")

# setup environment
env = setup_environment()
env.s = env.reset()
env.s = home_state
env.env.s = home_state
env.render_state(env.s)

# setup helper module for computing policies
policy_iteration = fhtw3dof_usecase_tools.iterations()

# setup object detection
tflistener = tf.TransformListener()

if load_pretrained:
    known_goalstates = np.loadtxt('known_goalstates.txt', dtype=int).tolist()
    policies = np.loadtxt('policies.txt', dtype=int).tolist()
    place_policy = np.loadtxt('place_policy.txt', dtype=int).tolist()
    home_policy = np.loadtxt('home_policy.txt', dtype=int).tolist()
else:
    known_goalstates = []
    policies = []
    # compute policy to place state
    env.P = env.reset_rewards(env.P, 0, env.nS)
    env.P, idx = env.set_state_reward(env.P, place_state)
    if idx < 1:
                print("Place state not reachable.")
                quit()
    place_policy = policy_iteration.policy_iteration(env)
    # compute policy to home state
    env.P = env.reset_rewards(env.P, 0, env.nS)
    env.P, idx = env.set_state_reward(env.P, home_state)
    if idx < 1:
                print("Home state not reachable.")
                quit()
    home_policy = policy_iteration.policy_iteration(env)

#init hebi control class and sync virtual and real robots
if real:
    hebi_pub = hebi_publisher.hebi_publisher()
    #hebi_pub.publish_pose_to_robot()

while True:
    #trigger loop
    try:
        int(input())
    except ValueError:
        break
    #input goalstate    
    goal_state, j3_goal, j2_goal, j1_goal = compute_goal(env, goalstate_distance)
    #check if goalstate is valid
    if goal_state < 0 or goal_state >= env.nS:
        print("Goalstate is not within the Robot's State-Size")
        continue
    if env.Colliding_States[goal_state]:
        print("Goalstate is Colliding with Cbstacles.")
        continue
    #check if the goalstate has already been trained
    try:
        idx = known_goalstates.index(goal_state)
    except ValueError as err:
        #reset env goal
        env.P = env.reset_rewards(env.P, 0, env.nS)
        #set goalpose
        env.P, idx = env.set_state_reward(env.P, goal_state)
        if idx < 1:
            print("Goalstate not reachable.")
            continue
        #compute policy to goal
        tmp_policy = policy_iteration.policy_iteration(env)
        #append to lists of known goalstates and policies
        known_goalstates.append(goal_state)
        policies.append(tmp_policy)
        idx = len(known_goalstates)-1
    # apply policy to current goal
    r  = apply_policy(policies[idx], goal_state, env)
    # go to grasp position
    goto_jointstate(j3_goal, j2_goal, j1_goal, env)
    # perform trajectory on real robot
    if real: hebi_pub.publish_trajectory_to_robot()
    time.sleep(8)
    # Close gripper
    print("Closing gripper")
    if real: hebi_pub.close_gripper()
    time.sleep(5)
    # Go to place state
    r = apply_policy(place_policy, place_state, env)
    if real: hebi_pub.publish_trajectory_to_robot()
    time.sleep(8)
    # Open gripper
    print("Opening gripper")
    if real: hebi_pub.open_gripper()
    time.sleep(5)
    # Go to home state
    r = apply_policy(home_policy, home_state, env)
    if real: hebi_pub.publish_trajectory_to_robot()
    time.sleep(8)

if save_model:
    np.savetxt('home_policy.txt', home_policy,fmt='%d')
    np.savetxt('place_policy.txt', place_policy,fmt='%d')
    np.savetxt('policies.txt', policies,fmt='%d')
    np.savetxt('known_goalstates.txt', known_goalstates,fmt='%d')
    print("Policies saved successfully.")
