#! /usr/bin/env python
import roslib; roslib.load_manifest('compass_biped')
import math, rospy, os, rosparam
from RobotController.msg import * 
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from numpy import zeros, array, linspace, arange
import numpy as np
from JointController import JointCommands_msg_handler
from robot_state import robot_state
from math import ceil
import yaml
from copy import copy
from std_srvs.srv import Empty
import sys

class CB_Controller(object):
    """CB_Controller"""
    def __init__(self, arg):
        # super(CB_Controller, self).__init__()

        ##################################################################
        ######################## GAIT PARAMETERS #########################
        ##################################################################
        if len(arg)<12:
            self.SwingEff0 = 0.46        # Default swing effort
            self.kAp = 2                 # Aperture feedback gain
            self.DesAp = 0.4             # Desired aperture
            self.SupAnkGain_p = 10       # Ankle gain p during support
            self.SupAnkGain_d = 4        # Ankle gain d during support
            self.SupAnkSetPoint = -0.02  # Support ankle set point
            self.ToeOffEff = 4           # Toe-off effort
            self.ToeOffDur = 0.12        # Toe-off duration
            self.TotSwingDur = 0.85      # Swing duration
            self.FootExtEff = 0.95       # Foot extension effort (right before touch down)
            self.FootExtDur = 0.18       # Foot extension duration
            self.FreeSwingDur = 0.15     # Swing duration
        else:
            self.SwingEff0 = arg[0]      # Default swing effort
            self.kAp = arg[1]            # Aperture feedback gain
            self.DesAp = arg[2]          # Desired aperture
            self.SupAnkGain_p = arg[3]   # Ankle gain p during support
            self.SupAnkGain_d = arg[4]   # Ankle gain d during support
            self.SupAnkSetPoint = arg[5] # Support ankle set point
            self.ToeOffEff = arg[6]      # Toe-off effort
            self.ToeOffDur = arg[7]      # Toe-off duration
            self.TotSwingDur = arg[8]    # Swing duration
            self.FootExtEff = arg[9]     # Foot extension effort (right before touch down)
            self.FootExtDur = arg[10]    # Foot extension duration
            self.FreeSwingDur = arg[11]  # Swing duration
        
        self.StepsTaken = 0
        self.ApertureMean = 0
        self.ApertureStd = 0
        self.GlobalPos = 0

        self.reset_srv = rospy.ServiceProxy('/gazebo/reset_models', Empty)

        ##################################################################
        ###################### RAISE FOOT SEQUENCE #######################
        ##################################################################

        self.pos1=zeros(5)

        # Sequence 1 for raising the inner foot (from home position)
        self.seq1_1=copy(self.pos1)
        self.seq1_1[1] = self.seq1_1[3] = self.seq1_1[0] = 0.2

        self.seq1_2=copy(self.seq1_1)
        self.seq1_2[0] = -1.45
        self.seq1_2[1] = self.seq1_2[3] = 0.16
        self.seq1_2[2] = self.seq1_2[4] = 0.06

        self.seq1_3=copy(self.seq1_2)
        self.seq1_3[1] = self.seq1_3[3] = 0.1

        # Sequence 2 for raising the outer feet (from home position)
        self.seq2_1=copy(self.pos1)
        self.seq2_1[1] = self.seq2_1[3] = -0.2
        self.seq2_1[2] = self.seq2_1[4] = 0.2

        self.seq2_2=copy(self.seq2_1)
        self.seq2_2[2] = self.seq2_2[4] = -1.45
        self.seq2_2[1] = self.seq2_2[3] = -0.16
        self.seq2_2[0] = 0.06

        self.seq2_3=copy(self.seq2_2)
        self.seq2_3[1] = self.seq2_3[3] = -0.1

        ##################################################################
        ########################## INITIALIZE ############################
        ##################################################################

        # Initialize joint commands handler
        self._robot_name = 'compass_biped'
        self._jnt_names = ['inner_ankle','left_hip','left_ankle','right_hip','right_ankle']
        self.JC = JointCommands_msg_handler(self._robot_name,self._jnt_names)

        # Initialize robot state listener
        self.RS = robot_state(self._jnt_names)
        self.MsgSub = rospy.Subscriber('/'+self._robot_name+'/robot_state',RobotState,self.RS_cb)
        self.OdomSub = rospy.Subscriber('/ground_truth_odom',Odometry,self.Odom_cb)


    ##################################################################
    ########################### FUNCTIONS ############################
    ##################################################################

    def RaiseLeg(self,which):
        if which == "inner":
            self.JC.send_pos_traj(self.pos1,self.seq1_1,1,0.01)
            rospy.sleep(0.2)
            self.JC.send_pos_traj(self.seq1_1,self.seq1_2,1,0.01)
            self.JC.set_eff('inner_ankle',-5)
            self.JC.send_pos_traj(self.seq1_2,self.seq1_3,1,0.01)
            rospy.sleep(0.2)
        if which == "outer":
            self.JC.send_pos_traj(self.pos1,self.seq2_1,1,0.01)
            rospy.sleep(0.2)
            self.JC.send_pos_traj(self.seq2_1,self.seq2_2,1,0.01)
            self.JC.set_eff('left_ankle',-5)
            self.JC.set_eff('right_ankle',-5)
            self.JC.send_pos_traj(self.seq2_2,self.seq2_3,1,0.01)
            rospy.sleep(0.2)

    def TakeFirstStep(self,which):
        if which == "inner":
            self.JC.set_eff('left_ankle',-1)
            self.JC.set_eff('right_ankle',-1)
            self.JC.set_eff('left_hip',3.5)
            self.JC.set_eff('right_hip',3.5)
            self.JC.set_eff('inner_ankle',0)
            self.JC.send_command()
            rospy.sleep(0.67)
            self.JC.set_eff('left_ankle',0.5)
            self.JC.set_eff('right_ankle',0.5)
            self.JC.set_eff('left_hip',0)
            self.JC.set_eff('right_hip',0)
            self.JC.set_eff('inner_ankle',0)
            self.JC.send_command()

        if which == "outer":
            self.JC.set_eff('left_ankle',-3)
            self.JC.set_eff('right_ankle',-3)
            self.JC.set_eff('left_hip',-2.7)
            self.JC.set_eff('right_hip',-2.7)
            self.JC.set_pos('inner_ankle',0.1)
            self.JC.send_command()
            rospy.sleep(0.1)
            #JC.set_pos('inner_ankle',0.0)
            self.JC.set_eff('left_ankle',0.1)
            self.JC.set_eff('right_ankle',0.1)
            self.JC.send_command()
            rospy.sleep(0.55)
            self.JC.set_eff('left_hip',0)
            self.JC.set_eff('right_hip',0)
            self.JC.set_eff('inner_ankle',0.5)
            self.JC.send_command()
            rospy.sleep(0.1)
            self.JC.set_eff('left_ankle',0)
            self.JC.set_eff('right_ankle',0)
            self.JC.send_command()

    def TakeStep(self,which):
        #TimeElapsed = rospy.Time.now().to_sec() - self.StartTime

        Aperture = abs(self.RS.GetJointPos('left_hip'))
        SwingEff = self.SwingEff0 + self.kAp*(self.DesAp - Aperture)
            
        #if TimeElapsed<30:
        #    Aperture = abs(self.RS.GetJointPos('left_hip'))
        #    SwingEff = self.SwingEff0 + self.kAp*(self.DesAp - Aperture)
        #    #print SwingEff
        #else:
        #    if which == "outer":
        #        SwingEff = 0.20
        #    else:
        #        SwingEff = 0.28
        if which == "outer":
            # Toe off
            self.JC.set_gains('inner_ankle',self.SupAnkGain_p,self.SupAnkGain_d,0,set_default = False)
            self.JC.set_pos('inner_ankle',self.SupAnkSetPoint)
            self.JC.set_eff('left_ankle',self.ToeOffEff)
            self.JC.set_eff('right_ankle',self.ToeOffEff)
            self.JC.set_eff('left_hip',0)
            self.JC.set_eff('right_hip',0)
            self.JC.send_command()
            rospy.sleep(self.ToeOffDur)
            # Raise feet and swing
            self.JC.set_eff('left_ankle',-3)
            self.JC.set_eff('right_ankle',-3)
            self.JC.set_eff('left_hip',-SwingEff)
            self.JC.set_eff('right_hip',-SwingEff)
            self.JC.send_command()
            rospy.sleep(self.TotSwingDur-self.FootExtDur-self.FreeSwingDur)
            # Lower feet
            self.JC.set_eff('left_ankle',self.FootExtEff)
            self.JC.set_eff('right_ankle',self.FootExtEff)
            self.JC.send_command()
            rospy.sleep(self.FootExtDur)
            # End swing
            self.JC.set_eff('left_hip',0)
            self.JC.set_eff('right_hip',0)
            self.JC.set_eff('left_ankle',0)
            self.JC.set_eff('right_ankle',0)
            self.JC.send_command()
            rospy.sleep(self.FreeSwingDur)

        if which == "inner":
            # Toe off
            self.JC.set_gains('left_ankle',self.SupAnkGain_p,self.SupAnkGain_d,0,set_default = False)
            self.JC.set_gains('right_ankle',self.SupAnkGain_p,self.SupAnkGain_d,0,set_default = False)
            self.JC.set_pos('left_ankle',self.SupAnkSetPoint)
            self.JC.set_pos('right_ankle',self.SupAnkSetPoint)
            self.JC.set_eff('inner_ankle',self.ToeOffEff)
            self.JC.set_eff('left_hip',0)
            self.JC.set_eff('right_hip',0)
            self.JC.send_command()
            rospy.sleep(self.ToeOffDur)
            # Raise feet and swing
            self.JC.set_eff('inner_ankle',-3)
            self.JC.set_eff('left_hip',SwingEff)
            self.JC.set_eff('right_hip',SwingEff)
            self.JC.send_command()
            rospy.sleep(self.TotSwingDur-self.FootExtDur-self.FreeSwingDur)
            # Lower feet
            self.JC.set_eff('inner_ankle',self.FootExtEff)
            self.JC.send_command()
            rospy.sleep(self.FootExtDur)
            # End swing
            self.JC.set_eff('left_hip',0)
            self.JC.set_eff('right_hip',0)
            self.JC.set_eff('inner_ankle',0)
            self.JC.send_command()
            rospy.sleep(self.FreeSwingDur)

    def ResetPose(self):
        self.JC.set_pos('left_hip',0)
        self.JC.set_pos('left_ankle',0)
        self.JC.set_pos('right_hip',0)
        self.JC.set_pos('right_ankle',0)
        self.JC.set_pos('inner_ankle',0)
        self.JC.send_command()

    def RS_cb(self,msg):

        self.RS.UpdateState(msg)

    def Odom_cb(self,msg):
        self.GlobalPos = msg.pose.pose.position

    def reset(self):
        self.reset_srv()
        rospy.sleep(0.5)

        while self.GlobalPos.z<0.95 or self.GlobalPos.z>1.05 or abs(self.GlobalPos.x)>0.5:
            self.reset_srv()
            rospy.sleep(0.5)

    def Run(self,TimeOut = 0):
        rospy.sleep(0.1) 
        # Start with inner foot raised
        self.JC.send_pos_traj(self.RS.GetJointPos(),self.seq2_3,0.5,0.01)
        self.reset()
        rospy.sleep(1) 

        self.StartTime = rospy.Time.now().to_sec()

        self.TakeFirstStep("outer")
        self.ApertureMean = abs(self.RS.GetJointPos('left_hip'))

        self.Leg = "inner"
        self.Go=1
        while self.Go == 1:
            TimeElapsed = rospy.Time.now().to_sec() - self.StartTime

            if self.GlobalPos.z<0.6 or self.GlobalPos.z>1.4 or (TimeElapsed>TimeOut and TimeOut>0):
                # if the robot fell, stop running and return fitness
                return self.GlobalPos, self.ApertureStd

            self.TakeStep(self.Leg)

            self.StepsTaken += 1
            ThisAperture = abs(self.RS.GetJointPos('left_hip'))
            self.ApertureMean += (ThisAperture-self.ApertureMean) / self.StepsTaken
            self.ApertureStd += ((ThisAperture-self.ApertureMean)**2 - self.ApertureStd) / self.StepsTaken

            if self.Leg == "inner":
                self.Leg = "outer"
            else:
                self.Leg = "inner"
    def __del__(self):
        self.MsgSub.unregister()
        self.OdomSub.unregister()




##################################################################
######################### USAGE EXAMPLE ##########################
##################################################################
def main(lst):
    # for k in xrange(10):
    CBC = CB_Controller(lst)
    A,B = CBC.Run(30)
    del CBC
    out_file = open('last_score.txt','w')
    out_file.write(str(A.x)+'\n')
    out_file.write(str(A.y)+'\n')
    out_file.write(str(A.z)+'\n')
    out_file.write(str(B))
    rospy.sleep(1)


if __name__=='__main__':
    lst = []
    for k in sys.argv[1:]:
        lst.append(float(k))
    main(lst)

