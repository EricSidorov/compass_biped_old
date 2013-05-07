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

class GCB_Controller(object):
    """Ghost CB_Controller for the compas biped model that uses 2 overlapping legs"""
    def __init__(self, arg):
        super(GCB_Controller, self).__init__()

        ##################################################################
        ######################## GAIT PARAMETERS #########################
        ##################################################################
        if len(arg)<12:
            self.SwingEff0 = 0.45        # Default swing effort
            self.kAp = 2                 # Aperture feedback gain
            self.DesAp = 0.35            # Desired aperture
            self.SupAnkGain_p = 10       # Ankle gain p during support
            self.SupAnkGain_d = 5        # Ankle gain d during support
            self.SupAnkSetPoint = -0.03  # Support ankle set point
            self.ToeOffEff0 = 4.0        # Toe-off effort
            self.ToeOffDur = 0.16        # Toe-off duration
            self.TotSwingDur = 0.85      # Swing duration
            self.FootExtEff = 0.85       # Foot extension effort (right before touch down)
            self.FootExtDur = 0.18       # Foot extension duration
            self.FreeSwingDur = 0.15     # Swing duration
            
            self.SwingEff = self.SwingEff0
            self.ToeOffEff = self.ToeOffEff0
        else:
            self.SwingEff0 = arg[0]      # Default swing effort
            self.kAp = arg[1]            # Aperture feedback gain
            self.DesAp = arg[2]          # Desired aperture
            self.SupAnkGain_p = arg[3]   # Ankle gain p during support
            self.SupAnkGain_d = arg[4]   # Ankle gain d during support
            self.SupAnkSetPoint = arg[5] # Support ankle set point
            self.ToeOffEff0 = arg[6]      # Toe-off effort
            self.ToeOffDur = arg[7]      # Toe-off duration
            self.TotSwingDur = arg[8]    # Swing duration
            self.FootExtEff = arg[9]     # Foot extension effort (right before touch down)
            self.FootExtDur = arg[10]    # Foot extension duration
            self.FreeSwingDur = arg[11]  # Swing duration
            
            self.SwingEff = self.SwingEff0
            self.ToeOffEff = self.ToeOffEff0
        
        self.StepsTaken = 0
        self.ApertureMean = 0
        self.ApertureStd = 0
        self.GlobalPos = 0

        self.reset_srv = rospy.ServiceProxy('/gazebo/reset_models', Empty)

        ##################################################################
        ###################### RAISE FOOT SEQUENCE #######################
        ##################################################################

        self.pos1=zeros(3)

        # Sequence 1 for raising the left foot (from home position)
        self.seq1_1=copy(self.pos1)
        self.seq1_1[0] = self.seq1_1[1] = 0.2

        self.seq1_2=copy(self.seq1_1)
        self.seq1_2[1] = -1.45
        self.seq1_2[0] = 0.16
        self.seq1_2[2] = 0.06

        self.seq1_3=copy(self.seq1_2)
        self.seq1_3[0] = -0.1

        # Sequence 2 for raising the right foot (from home position)
        self.seq2_1=copy(self.pos1)
        self.seq2_1[0] = -0.2
        self.seq2_1[2] = 0.2

        self.seq2_2=copy(self.seq2_1)
        self.seq2_2[2] = -1.45
        self.seq2_2[0] = -0.16
        self.seq2_2[1] = 0.06

        self.seq2_3=copy(self.seq2_2)
        self.seq2_3[0] = 0.1

        ##################################################################
        ########################## INITIALIZE ############################
        ##################################################################

        # Initialize joint commands handler
        self._robot_name = "ghost_compass_biped"
        self._jnt_names = ["hip","left_ankle","right_ankle"]
        self.JC = JointCommands_msg_handler(self._robot_name,self._jnt_names)

        # Initialize robot state listener
        self.RS = robot_state(self._jnt_names)
        self.MsgSub = rospy.Subscriber('/'+self._robot_name+'ghost_compass_biped/robot_state',RobotState,self.RS_cb)
        self.OdomSub = rospy.Subscriber('/ground_truth_odom',Odometry,self.Odom_cb)


    ##################################################################
    ########################### FUNCTIONS ############################
    ##################################################################

    def RaiseLeg(self,which):
        if which == "left":
            self.JC.send_pos_traj(self.pos1,self.seq1_1,1,0.01)
            rospy.sleep(0.2)
            self.JC.send_pos_traj(self.seq1_1,self.seq1_2,1,0.01)
            self.JC.set_eff('left_ankle',-5)
            self.JC.send_pos_traj(self.seq1_2,self.seq1_3,1,0.01)
            rospy.sleep(0.2)
        if which == "right":
            self.JC.send_pos_traj(self.pos1,self.seq2_1,1,0.01)
            rospy.sleep(0.2)
            self.JC.send_pos_traj(self.seq2_1,self.seq2_2,1,0.01)
            self.JC.set_eff('right_ankle',-5)
            self.JC.send_pos_traj(self.seq2_2,self.seq2_3,1,0.01)
            rospy.sleep(0.2)

    def TakeFirstStep(self,which):
        if which == "right":
            self.JC.set_eff('right_ankle',-3)
            self.JC.set_eff('hip',5.5)
            self.JC.set_eff('left_ankle',0.2)
            self.JC.send_command()
            rospy.sleep(0.1)
            self.JC.set_eff('right_ankle',0)
            self.JC.send_command()
            rospy.sleep(0.55)
            self.JC.set_eff('hip',0)
            self.JC.set_eff('left_ankle',1)
            self.JC.send_command()
            rospy.sleep(0.1)

        if which == "left":
            self.JC.set_eff('left_ankle',-3)
            self.JC.set_eff('hip',-5.5)
            self.JC.set_pos('right_ankle',0.2)
            self.JC.send_command()
            rospy.sleep(0.1)
            self.JC.set_eff('left_ankle',0)
            self.JC.send_command()
            rospy.sleep(0.55)
            self.JC.set_eff('hip',0)
            self.JC.set_eff('right_ankle',1)
            self.JC.send_command()
            rospy.sleep(0.1)

    def TakeStep(self,which):
        TimeElapsed = rospy.Time.now().to_sec() - self.StartTime

        if TimeElapsed<60:
            Aperture = abs(self.RS.GetJointPos('hip'))
            self.SwingEff = self.SwingEff0 + self.kAp*(self.DesAp - Aperture)
			
            print Aperture
        else:
            Aperture = abs(self.RS.GetJointPos('hip'))
            #self.ToeOffEff = self.ToeOffEff0 - 2*(self.DesAp - Aperture)
            #print self.ToeOffEff
            #print self.SwingEff
        if which == "right":
            # Toe off
            self.JC.set_gains('left_ankle',self.SupAnkGain_p,self.SupAnkGain_d,0,set_default = False)
            self.JC.set_pos('left_ankle',self.SupAnkSetPoint)
            self.JC.set_eff('right_ankle',self.ToeOffEff)
            self.JC.set_eff('hip',0)
            self.JC.send_command()
            rospy.sleep(self.ToeOffDur)
            # Raise feet and swing
            self.JC.set_eff('right_ankle',-3)
            self.JC.set_eff('hip',self.SwingEff)
            self.JC.send_command()
            rospy.sleep(self.TotSwingDur-self.FootExtDur-self.FreeSwingDur)
            # Lower feet
            self.JC.set_eff('right_ankle',self.FootExtEff)
            self.JC.send_command()
            rospy.sleep(self.FootExtDur)
            # End swing
            self.JC.set_eff('hip',0)
            self.JC.set_eff('right_ankle',0)
            self.JC.send_command()
            rospy.sleep(self.FreeSwingDur)

        if which == "left":
            # Toe off
            self.JC.set_gains('right_ankle',self.SupAnkGain_p,self.SupAnkGain_d,0,set_default = False)
            self.JC.set_pos('right_ankle',self.SupAnkSetPoint)
            self.JC.set_eff('left_ankle',self.ToeOffEff)
            self.JC.set_eff('hip',0)
            self.JC.send_command()
            rospy.sleep(self.ToeOffDur)
            # Raise feet and swing
            self.JC.set_eff('left_ankle',-3)
            self.JC.set_eff('hip',-self.SwingEff)
            self.JC.send_command()
            rospy.sleep(self.TotSwingDur-self.FootExtDur-self.FreeSwingDur)
            # Lower feet
            self.JC.set_eff('left_ankle',self.FootExtEff)
            self.JC.send_command()
            rospy.sleep(self.FootExtDur)
            # End swing
            self.JC.set_eff('hip',0)
            self.JC.set_eff('left_ankle',0)
            self.JC.send_command()
            rospy.sleep(self.FreeSwingDur)

    def ResetPose(self):
        self.JC.set_pos('hip',0)
        self.JC.set_pos('left_ankle',0)
        self.JC.set_pos('right_ankle',0)
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

        # Start with right foot raised
        self.JC.send_pos_traj(self.RS.GetJointPos(),self.seq2_3,0.5,0.01)
        self.reset()
        rospy.sleep(1) 

        self.StartTime = rospy.Time.now().to_sec()

        self.TakeFirstStep("right")
        self.ApertureMean = abs(self.RS.GetJointPos('hip'))

        self.Leg = "left"
        self.Go=1
        while self.Go == 1:
            TimeElapsed = rospy.Time.now().to_sec() - self.StartTime

            if self.GlobalPos.z<0.6 or self.GlobalPos.z>1.4 or (TimeElapsed>TimeOut and TimeOut>0):
                # if the robot fell, stop running and return fitness
                return self.GlobalPos, self.ApertureStd

            self.TakeStep(self.Leg)

            self.StepsTaken += 1
            ThisAperture = abs(self.RS.GetJointPos('hip'))
            self.ApertureMean += (ThisAperture-self.ApertureMean) / self.StepsTaken
            self.ApertureStd += ((ThisAperture-self.ApertureMean)**2 - self.ApertureStd) / self.StepsTaken

            if self.Leg == "right":
                self.Leg = "left"
            else:
                self.Leg = "right"


##################################################################
######################### USAGE EXAMPLE ##########################
##################################################################

if __name__=='__main__':
    CBC = GCB_Controller([])
    A,B = CBC.Run(60)
    print A,B
