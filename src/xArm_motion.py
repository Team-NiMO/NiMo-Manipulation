#!/usr/bin/env python3

import sys
import numpy as np
import rospy
from geometry_msgs.msg import Pose

#API 
from xarm.wrapper import XArmAPI

#Perception
from stalk_detect.srv import GetStalk

#Manipulation
from nimo_manipulation.srv import (arc_move, go2corn, go2EM, home_pos, hook, unhook)


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

        self.get_xArm_service = rospy.Service('home_pos', home_pos, self.home_pos)
        self.get_xArm_service = rospy.Service('home_pos', arc_move, self.arc_motion)
        self.get_xArm_service = rospy.Service('home_pos', go2corn, self.go2corn)
        self.get_xArm_service = rospy.Service('home_pos', go2EM, self.go2EM)
        self.get_xArm_service = rospy.Service('home_pos', hook, self.hook)
        self.get_xArm_service = rospy.Service('home_pos', unhook, self.unhook)

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

        self.arm.set_servo_angle(angle=[0, -90, 0, 0, 0, 0], is_radian=False, wait=True)

    @classmethod
    def go2EM_plane(self):
        if VERBOSE:
            rospy.loginfo('Going to External Mechanisms Plane')
        self.arm.set_servo_angle(angle=[-90, 41.5, -40.3, 0, -88.3, 0], is_radian=False, wait=True)

    @classmethod
    def go2EM(self):
        '''
        xArm moves to the external mechanisms nozzles based on the ID number:

        Parameters:
            The request: req(get_xArm_service)
                - id = id number of the nozzle to move to
                     = id=1 -> water nozzle
                       id=2 -> lowconc nozzle
                       id=3 -> highconc nozzle
        
        Returns:
                - status = xArm status

        '''
    
        if VERBOSE: rospy.loginfo("Going to External Mechnanisms")
        self.go2EM()

        if self.id == 1:
            print("hi")

        # TODO: ID=1
        print(f"---- going to cleaning nozzle ----")
        ### TODO: MODIFY THIS TO INCLUDE NEW CODE FROM AMIGA ###
        self.arm.set_servo_angle(angle=[-133.6, 56.9, -53.4, 46, -85.6, 0], is_radian=False, wait=True)

        print(f"---- going to calibration nozzles ----")
        # TODO: ID=2
        self.arm.set_servo_angle(angle=[-118.3, 60.2, -69.9, 62.8, -77.9, -13.2], is_radian=False, wait=True)
        # TODO: ID=3
        self.arm.set_servo_angle(angle=[-110.4, 75.9, -107.8, 73, -74.5, -33.8], is_radian=False, wait=True)
        ### TODO: MODIFY THIS TO INCLUDE NEW CODE FROM AMIGA ###

    @classmethod
    def go2corn(self):
        if VERBOSE:
            rospy.loginfo("Going to the cornstalk")
        
        # [x, y, z] of the cornstalk (IN THE WORLD FRAME)
        corn_pose = self.get_stalk_pose()

        #TODO: Include the code of the the xArm moving to the vicinity of the cornstalk
        


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

    @classmethod
    def get_stalk_pose(self):
        if VERBOSE:
            rospy.loginfo("Getting the pose of the cornstalk")

        # Getting the pose of the cornstalk from the perception stack (using the get_stalk service)
        rospy.wait_for_service('get_stalk')
        get_stalk_service = rospy.ServiceProxy('get_stalk', GetStalk)

        try:
            s_pose = get_stalk_service(num_frames=1, timeout=30.0) #5 frames,20 sec
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

        print(' ---- Got response from stalk detection:', s_pose.position, ' ----')

        print("POSE IS", [s_pose.position.x, s_pose.position.y, s_pose.position.z])
        return  np.array([s_pose.position.x, s_pose.position.y, s_pose.position.z])