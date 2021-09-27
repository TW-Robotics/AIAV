#!/usr/bin/env python3

# Dieses Skript wendet inverse Kinematik an, um durch Policy Iteration verursachte Ungenauigkeiten auszugleichen.
#
# This code is available under a GPL v3.0 license and comes without
# any explicit or implicit warranty.
#
# (C) Simon Schwaiger 2021 <schwaige@technikum-wien.at>

import numpy as np
import sympy as sp
from scipy import linalg
import math
import time

import rospy
import tf
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, Pose, PointStamped, Point

""" Calculates the robot's inverse kinematics in order to get the jointstates needed to reach tcp x,y and z. """

# helper functions for kinematics
def c(a):
    return sp.cos(a)

def s(a):
    return sp.sin(a)

def acos(a):
    while(a > 1):
        a -= 2
    while(a < -1):
        a += 2
    return math.acos(a)

def lawOfCos(a, b, c):
    return math.acos((a**2 + b**2 - c**2)/(2*a*b))

def asin(a):
    return math.asin(a)

def atan(a):
    return math.atan(a)

# robot parameters are known
l1 = 0.086
l2 = 0.175
l3 = 0.16 + 0.1
o1 = 0.04

# dummy tcp for testing
#tcp_x = 0.227
#tcp_y = 0.31
#tcp_z = 0.072

#tcp_x = 0.212 
#tcp_y = -0.226 
#tcp_z = 0.096

def getJointstate(tcp_x, tcp_y, tcp_z):
    # calculate distance between tcp and 0 in the xy-axis
    tcp_dist = math.sqrt(tcp_x**2 + tcp_y**2)

    # calculate distance between P and 0
    p_dist = math.sqrt(tcp_dist**2 - o1**2)

    # calculate d1
    d1 = asin(o1/tcp_dist) + math.atan2(tcp_y,tcp_x)

    # calculate d2 and d3 based on the law of cosine

    y = tcp_z - l1
    x = p_dist

    j2j3_dist = math.sqrt(x**2 + y**2)

    d2 = -lawOfCos(j2j3_dist, l2, l3) + math.atan2(x, y) - math.pi/2
    d3 = lawOfCos(l2, l3, j2j3_dist) - math.pi

    print("joints: {} {} {}".format(d1, d2, d3))

    return round(d3,6), round(d2,6), round(d1,6)
