#!/usr/bin/env python3

import sys
import numpy as np
import rospy
import geometry_msgs.msg
import time
import tf2_ros

#API 
from xarm.wrapper import XArmAPI

#Perception
from stalk_detect.srv import GetStalk

#Manipulation
# from nimo_manipulation.srv import (arc_move, go2corn, go2EM, home_pos, hook, unhook)
from nimo_manipulation.srv import *


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
        rospy.loginfo('Starting xArm_motion node.')
        rospy.loginfo("Creating xArm_Wrapper for ip: {ip_addr} ----")
        self.ip = ip_addr
        # Init the tf Buffer counter to store transforms from the corn to the end effector
        self.counter = 0

        # self.initialize_xarm()

        try:
            self.get_xArm_service = rospy.Service('home_pos', home_pos, self.home_pos)
            self.get_xArm_service = rospy.Service('arc_move', arc_move, self.arc_motion)
            self.get_xArm_service = rospy.Service('go2corn', go2corn, self.go2corn)
            self.get_xArm_service = rospy.Service('go2EM', go2EM, self.go2EM)
            self.get_xArm_service = rospy.Service('hook', hook, self.hook)
            self.get_xArm_service = rospy.Service('unhook', unhook, self.unhook)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

        rospy.loginfo('Waiting for service calls...')

        __flag = None


    @classmethod
    def initialize_xarm(self):
        rospy.loginfo('Initializing the xArm')

        self.arm = XArmAPI(self.ip)
        self.arm.motion_enable(enable=True)
        self.arm.set_mode(0)
        self.arm.set_state(state=0)

    @classmethod 
    def home_pos(self, req: home_posResponse) -> home_posRequest:
        '''
        xArm moves to the home position:
        
        Returns:
                - status = xArm status
        '''

        rospy.loginfo('Going to Home Position')
        self.arm.set_servo_angle(angle=[0, -90, 0, 0, 0, 0], is_radian=False, wait=True)
        # return self.status
        return home_posResponse(status="at home position")


    @classmethod
    def go2EM_plane(self):
        '''
        xArm moves to the external mechanisms plane to support go2EM():
        '''
        # if VERBOSE:
        rospy.loginfo('Going to External Mechanisms Plane')
        self.arm.set_servo_angle(angle=[-90, 41.5, -40.3, 0, -88.3, 0], is_radian=False, wait=True)

    @classmethod
    def go2EM(self, req: go2EMResponse) -> go2EMRequest:
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

        rospy.loginfo("Going to External Mechnanisms")
        # self.go2EM_plane()
        
        if req.id == "clean":
            rospy.loginfo("Going to Cleaning Nozzle")
            print(f"---- going to cleaning nozzle ----")
            ### TODO: MODIFY THIS TO INCLUDE NEW CODE FROM AMIGA ###
            self.arm.set_servo_angle(angle=[-133.6, 56.9, -53.4, 46, -85.6, 0], is_radian=False, wait=True)
            action = "moved to cleaning nozzle"

        elif req.id == "cal_low":
            rospy.loginfo("Going to Low Calibration Nozzle")
            print(f"---- going to low calibration nozzle ----")
            self.arm.set_servo_angle(angle=[-118.3, 60.2, -69.9, 62.8, -77.9, -13.2], is_radian=False, wait=True)
            ### TODO: MODIFY THIS TO INCLUDE NEW CODE FROM AMIGA ###
            action = "moved to low calibration nozzle"

        elif req.id == "cal_high":
            rospy.loginfo("Going to High Calibration Nozzle")
            print(f"---- going to high calibration nozzle ----")
            self.arm.set_servo_angle(angle=[-110.4, 75.9, -107.8, 73, -74.5, -33.8], is_radian=False, wait=True)
            ### TODO: MODIFY THIS TO INCLUDE NEW CODE FROM AMIGA ###
            action = "moved to high calibration nozzle"
            # self.go2EM_plane()

        return go2EMResponse(status = "moved to nozzle: " + req.id)

    @classmethod
    def go2corn(self):
        # if VERBOSE:
        rospy.loginfo("Going to the cornstalk")
        
        # [x, y, z] of the cornstalk (IN THE WORLD FRAME)
        corn_pose = self.get_stalk_pose()

        #TODO: Include the code of the the xArm moving to the vicinity of the cornstalk

    @classmethod
    def hook(self, req: hookResponse) -> hookRequest:
        '''
        xArm moves to the hook the cornstalk at an angle provided by:

        Parameters:
            The request: req(get_xArm_service)
                - grasp points 
                     = [x, y, z] of the grasp points from the Perception stack
        
        Returns:
                - angle = best insertion angle for insertion
        '''
        rospy.loginfo("Going to hook the cornstalk")

        self.transform(req.g)


        

        print("grasp points:", req.g)
        print("\ninsertion angle:", req.angle)

        return hookResponse(status="Hooked cornstalk")
    
    @classmethod
    def transform(self, grasp):
        # grasp is an array [x,y,z] grasp point from the request
        if self.counter < 1:
            self.counter += 1
            rospy.loginfo("Executing transforms from corn to end-effector")

            #Add: transformation to TF tree
            broadcaster = tf2_ros.StaticTransformBroadcaster()
            static_transformStamped = geometry_msgs.msg.TransformStamped()

            #TF header reference
            ## used to add a frame "child_frame" for the camera that is of a quaternion offset from "parent_link"
            # FORMAT: sendTransform((x, y, z), (r_x, r_y, r_z, r_w), rospy.Time.now(), "child_frame", "parent_link")
            # frame is that of the cornstalk?? -- #TODO:CONFIRM THIS

            #TF header
            static_transformStamped.header.stamp = rospy.Time.now() # time at which the pose has been received
            static_transformStamped.header.frame_id = "camera_link" # parent_link
            static_transformStamped.child_frame_id = "corn_cam" # child_link

            #TF position
            static_transformStamped.transform.translation.x = grasp[0]  
            static_transformStamped.transform.translation.y = grasp[1]
            static_transformStamped.transform.translation.z = grasp[2] # Setting this translation terms to be that of the cornstalk grasp points

            #TF quaternion
            static_transformStamped.transform.rotation.x = 0.0
            static_transformStamped.transform.rotation.y = 0.0
            static_transformStamped.transform.rotation.z = 0.0
            static_transformStamped.transform.rotation.w = 1.0

            # Add the corn_cam link at an offset from the "camera_link"
            broadcaster.sendTransform(static_transformStamped)

            time.sleep(1)
            # DO REQ_DETECT AGAIN

        # if self.counter > 1 else: transition to next state
        # lookup TF transformation from corn to end effector
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)

        tf_lookup_done = False

        while not tf_lookup_done:
            try:
                trans = tfBuffer.lookup_transform('world', 'corn_cam', rospy.Time())
                tf_lookup_done = True
                print(f" ******** TF lookup done ******** ")
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                # print(f" ******** TF lookup error ******** ")
                tf_lookup_done = False

            print(f" TF transformation trans x: {trans.transform.translation.x}, y: {trans.transform.translation.y}, z: {trans.transform.translation.z}")

            #publish TF for world to corn_cam
            broadcaster = tf2_ros.StaticTransformBroadcaster()
            static_transformStamped = geometry_msgs.msg.TransformStamped()

            #TF header
            static_transformStamped.header.stamp = rospy.Time.now()
            static_transformStamped.header.frame_id = "world"
            static_transformStamped.child_frame_id = "corn"
            #TF position
            static_transformStamped.transform.translation.x = trans.transform.translation.x
            static_transformStamped.transform.translation.y = trans.transform.translation.y
            static_transformStamped.transform.translation.z = trans.transform.translation.z
  
            #TF quaternion
            static_transformStamped.transform.rotation.x = 0.0
            static_transformStamped.transform.rotation.y = 0.0
            static_transformStamped.transform.rotation.z = 0.0
            static_transformStamped.transform.rotation.w = 1.0

            broadcaster.sendTransform(static_transformStamped)

            time.sleep(1)
            # No return value: #DO GO2_CORN



    @classmethod
    def unhook(self):
        '''
        xArm performs the reverse of the hook motion:
        
        Returns:
                - status = xArm status
        '''
        # if VERBOSE:
        rospy.loginfo("Going to unhook the cornstalk")

    @classmethod
    def arc_motion(self):
        # if VERBOSE:
        rospy.loginfo("Performing the arc motion")

    '''
    ##########################################################
    ###      VERIFY IF THE get_stalk_pose IS REQUIRED      ###
    ##########################################################
    '''
    @classmethod
    def get_stalk_pose(self):
        rospy.loginfo("Getting the pose of the cornstalk")

        # Getting the pose of the cornstalk from the perception stack (using the get_stalk service)
        rospy.wait_for_service('get_stalk')
        # returns the grasp points of the cornstalk
        get_stalk_service = rospy.ServiceProxy('get_stalk', GetStalk)

        try:
            s_pose = get_stalk_service(num_frames=1, timeout=30.0) #5 frames,20 sec
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

        print('---- Got response from stalk detection:', s_pose.position, ' ----')

        print("POSE IS", [s_pose.position.x, s_pose.position.y, s_pose.position.z])
        return  np.array([s_pose.position.x, s_pose.position.y, s_pose.position.z])
    

if __name__ == '__main__':
    rospy.init_node('nimo_manipulation')
    detect_node = xArm_Motion('192.168.1.214')
    rospy.spin()