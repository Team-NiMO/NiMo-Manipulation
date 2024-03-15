#!/usr/bin/env python3

import sys
import numpy as np
import rospy
from geometry_msgs.msg import Pose

#API 
from xarm.wrapper import XArmAPI

#Perception
from stalk_detect.srv import GetStalk

'''
Services involved in this package:
1. arc_move
2. go2corn
3. go2EM
4. home_pos
5. hook
6. unhook
'''


class xArm_Motion():
    @classmethod
    def __init__(self, ip_addr):
        if VERBOSE:
            rospy.loginfo('Starting xArm_motion node.')
        rospy.loginfo("Creating xArm_Wrapper for ip: {ip_addr} ----")
        self.ip = ip_addr

        if VERBOSE:
            rospy.loginfo('Waiting for service calls...')

    @classmethod
    def initialize_xarm(self):
        if VERBOSE:
            rospy.loginfo('Initializing the xArm')

        self.arm = XArmAPI(self.ip)
        self.arm.motion_enable(enable=True)
        self.arm.set_mode(0)
        self.arm.set_state(state=0)

    @classmethod 
    def home_pos(self):
        if VERBOSE:
            rospy.loginfo('Going to Home Position')
        '''
        self.get_xArm_service = rospy.Service('home_pos', home_pos, )
        '''
        self.arm.set_servo_angle(angle=[0, -90, 0, 0, 0, 0], is_radian=False, wait=True)

    @classmethod
    def go2EM_plane(self):
        if VERBOSE:
            rospy.loginfo('Going to External Mechanisms Plane')
        self.arm.set_servo_angle(angle=[-90, 41.5, -40.3, 0, -88.3, 0], is_radian=False, wait=True)

    @classmethod
    def go2EM(self):
        if VERBOSE: 
            rospy.loginfo("Going to External Mechnanisms")
        self.go2EM()

        print(f"---- going to cleaning nozzle ----")
        ### TO DO: MODIFY THIS TO INCLUDE NEW CODE FROM AMIGA ###
        self.arm.set_servo_angle(angle=[-133.6, 56.9, -53.4, 46, -85.6, 0], is_radian=False, wait=True)
        ### TO DO: MODIFY THIS TO INCLUDE NEW CODE FROM AMIGA ###

        print(f"---- going to calibration nozzles ----")
        ### TO DO: MODIFY THIS TO INCLUDE NEW CODE FROM AMIGA ###
        self.arm.set_servo_angle(angle=[-118.3, 60.2, -69.9, 62.8, -77.9, -13.2], is_radian=False, wait=True)
        self.arm.set_servo_angle(angle=[-110.4, 75.9, -107.8, 73, -74.5, -33.8], is_radian=False, wait=True)
        ### TO DO: MODIFY THIS TO INCLUDE NEW CODE FROM AMIGA ###

    @classmethod
    def go2corn(self):
        if VERBOSE:
            rospy.loginfo("Going to the cornstalk")

    @classmethod
    def arc_motion(self):
        if VERBOSE:
            rospy.loginfo("Performing the arc motion")

    @classmethod
    def hook(self):
        if VERBOSE:
            rospy.loginfo("Going to hook the cornstalk")

    @classmethod
    def unhook(self):
        if VERBOSE:
            rospy.loginfo("Going to unhook the cornstalk")