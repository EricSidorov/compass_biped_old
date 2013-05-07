#! /usr/bin/env python
import roslib; roslib.load_manifest('compass_biped')
import math, rospy, os, rosparam
from robotcon_msgs.msg import * 
from sensor_msgs.msg import JointState
from osrf_msgs.msg import JointCommands
from nav_msgs.msg import Odometry
from numpy import zeros, array, linspace, arange
import numpy as np
from JointController import JointCommands_msg_handler
from robot_state import robot_state
from math import ceil
import yaml
from copy import copy
from std_srvs.srv import Empty

reset = rospy.ServiceProxy('/gazebo/reset_models', Empty)

pos1=zeros(5)

# Sequence 1 for raising the inner foot (from home position)
seq1_1=copy(pos1)
seq1_1[1] = seq1_1[3] = seq1_1[0] = 0.2

seq1_2=copy(seq1_1)
seq1_2[0] = -1.45
seq1_2[1] = seq1_2[3] = 0.16
seq1_2[2] = seq1_2[4] = 0.06

seq1_3=copy(seq1_2)
seq1_3[1] = seq1_3[3] = 0.1

# Sequence 2 for raising the outer feet (from home position)
seq2_1=copy(pos1)
seq2_1[1] = seq2_1[3] = -0.2
seq2_1[2] = seq2_1[4] = 0.2

seq2_2=copy(seq2_1)
seq2_2[2] = seq2_2[4] = -1.45
seq2_2[1] = seq2_2[3] = -0.16
seq2_2[0] = 0.06

seq2_3=copy(seq2_2)
seq2_3[1] = seq2_3[3] = -0.1

def RaiseLeg(which):
    if which == "inner":
        #JC.set_pos('right_hip',0.1)
        #JC.set_pos('left_hip',0.1)
        JC.send_pos_traj(pos1,seq1_1,1,0.01)
        #JC.send_command()
        rospy.sleep(0.2)
        JC.send_pos_traj(seq1_1,seq1_2,1,0.01)
        JC.set_eff('inner_ankle',-5)
        #JC.set_eff('left_hip',1)
        #JC.set_eff('right_hip',1)
        JC.send_pos_traj(seq1_2,seq1_3,1,0.01)
        #JC.set_pos('right_hip',0)
        #JC.set_pos('left_hip',0)
        rospy.sleep(0.2)
    if which == "outer":
        JC.send_pos_traj(pos1,seq2_1,1,0.01)
        rospy.sleep(0.2)
        JC.send_pos_traj(seq2_1,seq2_2,1,0.01)
        JC.set_eff('left_ankle',-5)
        JC.set_eff('right_ankle',-5)
        JC.send_pos_traj(seq2_2,seq2_3,1,0.01)
        rospy.sleep(0.2)

def TakeFirstStep(which):
    if which == "inner":
        JC.set_eff('left_ankle',-1)
        JC.set_eff('right_ankle',-1)
        JC.set_eff('left_hip',3.5)
        JC.set_eff('right_hip',3.5)
        JC.set_eff('inner_ankle',0)
        JC.send_command()
        rospy.sleep(0.67)
        JC.set_eff('left_ankle',0.5)
        JC.set_eff('right_ankle',0.5)
        JC.set_eff('left_hip',0)
        JC.set_eff('right_hip',0)
        JC.set_eff('inner_ankle',0)
        JC.send_command()

    if which == "outer":
        JC.set_eff('left_ankle',-3)
        JC.set_eff('right_ankle',-3)
        JC.set_eff('left_hip',-2.7)
        JC.set_eff('right_hip',-2.7)
        JC.set_pos('inner_ankle',0.1)
        JC.send_command()
        rospy.sleep(0.1)
        #JC.set_pos('inner_ankle',0.0)
        JC.set_eff('left_ankle',0.1)
        JC.set_eff('right_ankle',0.1)
        JC.send_command()
        rospy.sleep(0.55)
        JC.set_eff('left_hip',0)
        JC.set_eff('right_hip',0)
        JC.set_eff('inner_ankle',0.5)
        JC.send_command()
        rospy.sleep(0.1)
        JC.set_eff('left_ankle',0)
        JC.set_eff('right_ankle',0)
        JC.send_command()

def TakeStep(which):
    Aperture = abs(RS.GetJointPos('left_hip'))
    SwingStr = 0.46 + 2*(0.4 - Aperture)
    if which == "outer":
        # Toe off
        JC.set_gains('inner_ankle',10,4,0,set_default = False)
        JC.set_pos('inner_ankle',-0.02)
        JC.set_eff('left_ankle',4)
        JC.set_eff('right_ankle',4)
        JC.set_eff('left_hip',0)
        JC.set_eff('right_hip',0)
        JC.send_command()
        rospy.sleep(0.12)
        # Raise feet and swing
        JC.set_eff('left_ankle',-3)
        JC.set_eff('right_ankle',-3)
        JC.set_eff('left_hip',-SwingStr)
        JC.set_eff('right_hip',-SwingStr)
        JC.send_command()
        rospy.sleep(0.52)
        # Lower feet
        JC.set_eff('left_ankle',0.95)
        JC.set_eff('right_ankle',0.95)
        JC.send_command()
        rospy.sleep(0.18)
        # End swing
        JC.set_eff('left_hip',0)
        JC.set_eff('right_hip',0)
        JC.set_eff('left_ankle',0)
        JC.set_eff('right_ankle',0)
        JC.send_command()
        rospy.sleep(0.15)

    if which == "inner":
        # Toe off
        JC.set_gains('left_ankle',10,4,0,set_default = False)
        JC.set_gains('right_ankle',10,4,0,set_default = False)
        JC.set_pos('left_ankle',-0.02)
        JC.set_pos('right_ankle',-0.02)
        JC.set_eff('inner_ankle',4)
        JC.set_eff('left_hip',0)
        JC.set_eff('right_hip',0)
        JC.send_command()
        rospy.sleep(0.12)
        # Raise feet and swing
        JC.set_eff('inner_ankle',-3)
        JC.set_eff('left_hip',SwingStr)
        JC.set_eff('right_hip',SwingStr)
        JC.send_command()
        rospy.sleep(0.52)
        # Lower feet
        JC.set_eff('inner_ankle',0.95)
        JC.send_command()
        rospy.sleep(0.18)
        # End swing
        JC.set_eff('left_hip',0)
        JC.set_eff('right_hip',0)
        JC.set_eff('inner_ankle',0)
        JC.send_command()
        rospy.sleep(0.15)

def ResetPose():
    JC.set_pos('left_hip',0)
    JC.set_pos('left_ankle',0)
    JC.set_pos('right_hip',0)
    JC.set_pos('right_ankle',0)
    JC.set_pos('inner_ankle',0)
    JC.send_command()

def RS_cb(msg):
    RS.UpdateState(msg)

# Initialize joint commands handler
JC = JointCommands_msg_handler()

# Initialize robot state listener
RS = robot_state()
MsgSub = rospy.Subscriber('/compass_biped/robot_state',RobotState,RS_cb)

rospy.sleep(0.1) 
#JC.send_pos_traj(RS.GetJointPos(),pos1,0.5,0.01)
JC.send_pos_traj(RS.GetJointPos(),seq2_3,0.5,0.01)
reset()

#rospy.sleep(1) 
#RaiseLeg("inner")
rospy.sleep(1.5) 
TakeFirstStep("outer")

TakeStep("inner")
TakeStep("outer")

Go=1
while Go == 1:
    TakeStep("inner")
    TakeStep("outer")
