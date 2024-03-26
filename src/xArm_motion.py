#!/usr/bin/env python3

import sys
import numpy as np
import quaternion
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
        # self.cur_status = None

        self.initialize_xarm()

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

        # Tracking of status
        self.cur_status = None
        self.flag = None
        # for unhooking
        self.reverse_x = None 
        self.reverse_y = None
        # to keep track of the angle at which the xarm is facing the cornstalk
        self.angle = 0

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

        Status ID: "home"
        
        Returns:
                - status = xArm status
        '''

        rospy.loginfo('Going to Home Position')

        # check condition to verify if xArm should move through the ext. mechanisms plane
        if(self.flag == 1):
            self.go2EM_plane()

        # end-effector faces the left side of the amiga base
        # self.arm.set_servo_angle(angle=[0, -90, 0, -90, 90, 0], is_radian=False, wait=True)
        self.arm.set_servo_angle(angle=[0, -90, 0, -90, 90, 90], is_radian=False, wait=True)

        self.cur_status = "home" # internal tracker
        self.flag = 0 

        # Response to the FSM
        return home_posResponse(status= self.cur_status)

    @classmethod
    def go2EM_plane(self):
        '''
        xArm moves to the external mechanisms plane to support go2EM():
        '''
        
        rospy.loginfo('Going to External Mechanisms Plane')
        print("flag in the EM plane function:",self.flag)
        self.arm.set_servo_angle(angle=[-90, 41.5, -40.3, 0, -88.3, 0], is_radian=False, wait=True)

    @classmethod
    def go2EM(self, req: go2EMResponse) -> go2EMRequest:
        '''
        xArm moves to the external mechanisms nozzles based on the ID number:

        Status ID: "EM"
        Response to FSM: "moved to nozzle: " + req.id

        Parameters:
            The request: req(get_xArm_service)
                - id = id number of the nozzle to move to
                     = id=1 -> water nozzle
                       id=2 -> lowconc nozzle
                       id=3 -> highconc nozzle
        
        Returns:
                - status = xArm status
        '''

        rospy.loginfo("Going to External Mechanisms")
        
        # check condition to verify if xArm should move through ext. mechanisms
        if(self.flag == 0):
            self.go2EM_plane()
        
        if req.id == "clean":
            rospy.loginfo("Going to Cleaning Nozzle")
            print(f"---- going to cleaning nozzle ----")
            ### TODO: MODIFY THIS TO INCLUDE NEW CODE FROM AMIGA ###
            self.arm.set_servo_angle(angle=[-133.6, 56.9, -53.4, 46, -85.6, 0], is_radian=False, wait=True)

        elif req.id == "cal_low":
            rospy.loginfo("Going to Low Calibration Nozzle")
            print(f"---- going to low calibration nozzle ----")
            ### TODO: MODIFY THIS TO INCLUDE NEW CODE FROM AMIGA ###
            self.arm.set_servo_angle(angle=[-118.3, 60.2, -69.9, 62.8, -77.9, -13.2], is_radian=False, wait=True)

        elif req.id == "cal_high":
            rospy.loginfo("Going to High Calibration Nozzle")
            print(f"---- going to high calibration nozzle ----")
            ### TODO: MODIFY THIS TO INCLUDE NEW CODE FROM AMIGA ###
            self.arm.set_servo_angle(angle=[-110.4, 75.9, -107.8, 73, -74.5, -33.8], is_radian=False, wait=True)

        self.cur_status = "EM"
        self.flag = 1

        return go2EMResponse(status = "moved to nozzle: " + req.id)

    @classmethod
    def go2corn(self, req: go2cornResponse) -> go2cornRequest:
        '''
        xArm moves to the external mechanisms nozzles based on the ID number:

        Status ID: "moved to corn"

        Parameters:
            The request: req(get_xArm_service)
                - id = id number of the nozzle to move to
                     = id=1 -> water nozzle
                       id=2 -> lowconc nozzle
                       id=3 -> highconc nozzle
        
        Returns:
                - status = xArm status
        '''

        rospy.loginfo("Going to the cornstalk")
        
        # [x, y, z] of the cornstalk (IN THE WORLD FRAME)
        # corn_pose = self.get_stalk_pose()

        #TODO: Include the code of the the xArm moving to the vicinity of the cornstalk

        '''
        # 1. get grasp point
        '''

        # check condition to verify if xArm should move through ext. mechanisms
        if(self.flag == 1):
            self.go2EM_plane()

        grasp = req.g
        self.angle = 0

        #Add: transformation to TF tree
        broadcaster = tf2_ros.StaticTransformBroadcaster()
        static_transformStamped = geometry_msgs.msg.TransformStamped()

        #TF header
        static_transformStamped.header.stamp = rospy.Time.now() # time at which the pose has been received
        static_transformStamped.header.frame_id = "world" # parent_link
        static_transformStamped.child_frame_id = "corn_cam" # child_link
        print("child frame: ", static_transformStamped.child_frame_id)
        
        # TODO: Corn position is in world frame not camera frame
        static_transformStamped.transform.translation.x = grasp[0] # z in OpenCV frame
        static_transformStamped.transform.translation.y = grasp[1] # x in OpenCV frame
        static_transformStamped.transform.translation.z = grasp[2] # y in OpenCV frame
        # Setting the translation terms to be that of the cornstalk grasp points

        #TF quaternion
        static_transformStamped.transform.rotation.x = 0.0
        static_transformStamped.transform.rotation.y = 0.0
        static_transformStamped.transform.rotation.z = 0.0
        static_transformStamped.transform.rotation.w = 1.0

        # Add the corn_cam link at an offset from the "camera_link"
        broadcaster.sendTransform(static_transformStamped)
        time.sleep(1)

        self.transform()
        
        # receive translation values for xArm6
        move = self.transform_lookup()
        print("move:", move)
        if move is None:
            return hookResponse(status="Wait")

        '''
        # 2. get relative movement to the grasp point - translation only 
        '''
        x, y, z = move[0], move[1], move[2]

        self.arm.set_position_aa(axis_angle_pose=[x, y+0.15, z, 0, 0, 0], speed=50, relative=True, wait=True)

        self.cur_status = "moved to corn"

        # return go2cornResponse(status = self.cur_status)
    
    @classmethod
    def calculate_position(self, centre, radius, angle):
        x_t = centre[0] + radius * np.sin(np.radians(angle))
        y_t = centre[1] + radius * np.cos(np.radians(angle))
        # z_t = centre[2]
        # return target parameters
        # print("x_t, y_t, z_t", x_t, y_t, z_t)
        return [x_t, y_t]

    @classmethod
    def hook(self, req: hookResponse) -> hookRequest:
        '''
        xArm moves to the hook the cornstalk at an angle provided by:

        Status ID: "hooked"

        Parameters:
            The request: req(get_xArm_service)
                - grasp points 
                     = [x, y, z] of the grasp points from the Perception stack
        
        Returns:
                - angle = best insertion angle for insertion
        '''

        # ---------------------------------------------------------------------------------------------------------
        # ---------------------------------------------------------------------------------------------------------
        '''
        ### Init + Frame Transforms for the xArm6
        '''
        # check condition to verify if xArm should move through ext. mechanisms
        if(self.flag == 1):
            self.go2EM_plane()

        grasp = req.g

        #Add: transformation to TF tree
        broadcaster = tf2_ros.StaticTransformBroadcaster()
        static_transformStamped = geometry_msgs.msg.TransformStamped()

        #TF header reference
        ## used to add a frame "child_frame" for the camera that is of a quaternion offset from "parent_link"
        # FORMAT: sendTransform((x, y, z), (r_x, r_y, r_z, r_w), rospy.Time.now(), "child_frame", "parent_link")
        # frame is that of the cornstalk?? -- #TODO:CONFIRM THIS

        #TF header
        static_transformStamped.header.stamp = rospy.Time.now() # time at which the pose has been received
        static_transformStamped.header.frame_id = "world" # parent_link
        static_transformStamped.child_frame_id = "corn_cam" # child_link
        print("child frame: ", static_transformStamped.child_frame_id)
        
        #TF position -> TODO: check with Mark
        # TODO: Corn position is in world frame not camera frame
        static_transformStamped.transform.translation.x = grasp[0]
        static_transformStamped.transform.translation.y = grasp[1]
        static_transformStamped.transform.translation.z = grasp[2]
        # Setting the translation terms to be that of the cornstalk grasp points

        #TF quaternion
        static_transformStamped.transform.rotation.x = 0.0
        static_transformStamped.transform.rotation.y = 0.0
        static_transformStamped.transform.rotation.z = 0.0
        static_transformStamped.transform.rotation.w = 1.0

        # Add the corn_cam link at an offset from the "camera_link"
        broadcaster.sendTransform(static_transformStamped)
        time.sleep(1)

        self.transform()
        # ---------------------------------------------------------------------------------------------------------

        rospy.loginfo("Going to hook the cornstalk")

        print("Going to plane")
        # self.arm.set_servo_angle(angle=[0, -78.4, -21.1, 0, 10.4, -90], is_radian=False, wait=True)
        self.arm.set_servo_angle(angle=[-90, -90, 0, -90, 0, 0], is_radian=False, wait=True)
        # time.sleep(5)
        
        # print("position_aa: ", self.arm.get_position_aa())
        # position_aa:  (0, [155.261932, 0.0, 584.046875, 128.274937, 0.0, 126.275658])

        #0. move away from corn to prevent hitting during rotation
        #1. pivot to face corn
        # print("Going to rotated camera plane")
        # print(" ---- move Y away to prevent hitting corn during rotation  ----")
        # self.arm.set_position_aa(axis_angle_pose=[0, 35, 0, 0, 0, -90], relative=True, wait=True)
        # self.arm.set_position_aa(axis_angle_pose=[0, -50, 0, 0, 0, 0], relative=True, wait=True)
        # time.sleep(5)
        # self.arm.set_position_aa(axis_angle_pose=[0, 0, 0, 50, 0, 0], relative=True, wait=True)
        # # self.arm.set_position_aa(axis_angle_pose=[0, 35, 0, 0, 0, 0], relative=True, wait=True)
        
        # # self.arm.set_position_aa(axis_angle_pose=[0, -80, 0, 0, 0, 0], relative=True, wait=True)
        # time.sleep(5)

        # print(" ---- rotating EE -90 deg Y  ----")
        # self.arm.set_position_aa(axis_angle_pose=[0, 0, 0, 0, 0, -90], relative=True, wait=True)

        
        # receive translation values for xArm6
        move = self.transform_lookup()
        print("move:", move)
        if move is None:
            return hookResponse(status="Wait")

        # ---------------------------------------------------------------------------------------------------------

        #3. move to corn position using the trans value
        # self.go_to_stalk_pose(move[0], move[1], move[2])
        
        #TODO: Modify the variable names
        x_mm, y_mm, z_mm = move[0], move[1], move[2]

        # Tuned by Mark's implementation for a static environment
        # x_mm_tuned_offset = 29
        x_mm_tuned_offset = 0

        #80mm is roughly center of gripper to edge of C clamp, 5 is fine-tuned offset
        x_mm_gripper_width = 80+5 
        x_mm_with_gripper_offest = x_mm - x_mm_gripper_width - x_mm_tuned_offset

        # y_mm_tuned_offset = -32
        y_mm_tuned_offset = -20
        print(f"x_mm, x_mm_with_gripper_offest {x_mm, x_mm_with_gripper_offest}")

        x_mm_deeper_clamp_insert = 14
        x_mm_deeper_clamp_retract = 25

        z_mm_tuned = z_mm

        y_mm_overshoot = 8
        y_mm_funnel = 12

        print(" ---- going to stalk pose  ----")

        # '''
        print(" 1. move X align 1/10 ")
        # self.arm.set_position_aa(axis_angle_pose=[x_mm_with_gripper_offest, 0, 0, 0, 0, 0], relative=True, wait=True)
        self.arm.set_position_aa(axis_angle_pose=[x_mm_with_gripper_offest, 0, 0, 0, 0, 0], speed=50, relative=True, wait=True) #reverse
        # time.sleep(10)

        print(" 2. move Y approach  2/10")
        self.arm.set_position_aa(axis_angle_pose=[0, y_mm+y_mm_tuned_offset, 0, 0, 0, 0], speed=50, relative=True, wait=True)
        # time.sleep(10)
        
        print(" 2.5 move Y to compensate overshoot  2.5/10")
        self.arm.set_position_aa(axis_angle_pose=[0, y_mm_overshoot, 0, 0, 0, 0], speed=50, relative=True, wait=True)
        # time.sleep(10)
        
        print(" 3. move Z to down 3/10 with z: {z_mm_tuned}")
        self.arm.set_position_aa(axis_angle_pose=[0, 0, z_mm_tuned, 0, 0, 0], speed=50, relative=True, wait=True)
        # time.sleep(10)
        # sys.exit()

        print(f" 4. move X center w gripper 4/10")
        # self.arm.set_position_aa(axis_angle_pose=[-x_mm_gripper_width-x_mm_tuned_offset, 0, 0, 0, 0, 0], relative=True, wait=True)
        self.arm.set_position_aa(axis_angle_pose=[x_mm_gripper_width-x_mm_tuned_offset, 0, 0, 0, 0, 0], relative=True, wait=True) # reverse
        # time.sleep(10)
        # print(f"-x_mm_gripper_width-x_mm_tuned_offset: {-x_mm_gripper_width-x_mm_tuned_offset}")
        print(f"x_mm_gripper_width-x_mm_tuned_offset: {x_mm_gripper_width-x_mm_tuned_offset}") #reverse

        print(f" 5. move X go deeper 5/10")
        # self.arm.set_position_aa(axis_angle_pose=[-x_mm_deeper_clamp_insert, 0, 0, 0, 0, 0], relative=True, wait=True)
        self.arm.set_position_aa(axis_angle_pose=[x_mm_deeper_clamp_insert, 0, 0, 0, 0, 0], relative=True, wait=True) # reverse
        # time.sleep(5)
        # print(f"-x_mm_deeper_clamp_insert: {-x_mm_deeper_clamp_insert}")
        print(f"x_mm_deeper_clamp_insert: {x_mm_deeper_clamp_insert}") #reverse

        print(f" 6. move X to recenter 6/10")
        # self.arm.set_position_aa(axis_angle_pose=[+x_mm_deeper_clamp_retract, 0, 0, 0, 0, 0], relative=True, wait=True)
        self.arm.set_position_aa(axis_angle_pose=[-x_mm_deeper_clamp_retract, 0, 0, 0, 0, 0], relative=True, wait=True) # reverse
        # print(f"x_mm_deeper_clamp_retract: {x_mm_deeper_clamp_retract}")
        print(f"-x_mm_deeper_clamp_retract: {-x_mm_deeper_clamp_retract}") #reverse 
        # time.sleep(5)

        print(f" 6.5 move Y to get corn on edge of funnel 6.5/10")
        self.arm.set_position_aa(axis_angle_pose=[0, y_mm_funnel, 0, 0, 0, 0], relative=True, wait=True)
        # time.sleep(5)
        print(f"y_mm_funnel: {y_mm_funnel}")
        # '''
        time.sleep(1)
        
        print("grasp points:", grasp)
        print("\ninsertion angle:", req.move_angle)

        # values for unhooking the cornstalk
        # self.reverse_x = -1*(-x_mm_gripper_width-x_mm_tuned_offset)
        self.reverse_x = -1*(x_mm_gripper_width-x_mm_tuned_offset) #reverse
        self.reverse_y = -1*(y_mm+y_mm_tuned_offset)

        self.cur_status = "hooked"
        self.flag = 0

        return hookResponse(status=self.cur_status)
    
    @classmethod
    def transform(self):
        # grasp is an array [x,y,z] grasp point from the request
        
        rospy.loginfo("Executing transforms from corn to end-effector")

        # if self.counter > 1 else: transition to next state
        # lookup TF transformation from corn to end effector
        tfBuffer = tf2_ros.Buffer(rospy.Duration(3.0))
        tf2_ros.TransformListener(tfBuffer)

        tf_lookup_done = False

        while not tf_lookup_done:
            print("here")
            try:
                trans = tfBuffer.lookup_transform('world', 'corn_cam', rospy.Time(), rospy.Duration(5.0))
                print(f" ******** transform TF lookup done ******** ")
                tf_lookup_done = True
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                tf_lookup_done = False
                print(f" ******** transform TF lookup error ******** ")
            
            print(f" TF transformation trans x: {trans.transform.translation.x}, y: {trans.transform.translation.y}, z: {trans.transform.translation.z}")

            #publish TF for world to corn_cam
            broadcaster = tf2_ros.StaticTransformBroadcaster()
            static_transformStamped = geometry_msgs.msg.TransformStamped()

            #TF header
            static_transformStamped.header.stamp = rospy.Time.now()
            static_transformStamped.header.frame_id = "world"
            static_transformStamped.child_frame_id = "corn_cam"
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
    def transform_lookup(self):
        tfBuffer = tf2_ros.Buffer(rospy.Duration(3.0))
        tf2_ros.TransformListener(tfBuffer)
        tf_lookup_done = False

        while not tf_lookup_done:
            try:
                trans = tfBuffer.lookup_transform('corn_cam', 'gripper', rospy.Time(), rospy.Duration(3.0))
                trans_g2b = tfBuffer.lookup_transform('link_base', 'gripper', rospy.Time(), rospy.Duration(3.0))
                tf_lookup_done = True
                print(f" ******** transform_lookup TF lookup done ******** ")
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            # except Exception as e:
            # except (tf2_ros.LookupException):
                print(f" ******** transform_lookup TF lookup error ******** ")
                tf_lookup_done = False

        # print("need to translate :", trans.transform.translation)
        p = np.array([trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z])
        x = -p[0]
        y = -p[1]
        z = -p[2]

        # reject spurious cam pose by thresholding Y
        # if y > -0.10 or y < -0.75:
        #     rospy.loginfo(" ******** ERROR: GARBAGE CAM DETECTION. WAITING FOR SERVICE *******")
        
        # else:
        # hard coded for initial mock test setup
            # # ''' 
            # x = -0.03628206015960567 # in m
            # y = -0.1458352749289401
            # # '''

            # x = -0.05
            # y = -0.16375

            # z = -0.1441743369126926

            # x = -x #gripper x coord is opposite direction of robot base x coord
        # x = x #gripper x coord is opposite direction of robot base x coord #reverse
        # z = -z #gripper x coord is opposite direction of robot base x coord

        x_mm = x*1000 # in mm 
        y_mm = y*1000 
        z_mm = z*1000

        move = [x_mm, y_mm, z_mm]
        return move

    @classmethod
    def unhook(self, req: unhookResponse) -> unhookRequest:
        '''
        xArm performs the reverse of the hook motion:

        Status ID: "Unhooked"
        
        Returns:
                - status = xArm status
        '''
        rospy.loginfo("Going to unhook the cornstalk")

        # check condition to verify if xArm should move through ext. mechanisms
        if(self.flag == 1):
            self.go2EM_plane()

        y_mm_gripper_width = 10
        print(f" 8. move out Y 8/10")
        self.arm.set_position_aa(axis_angle_pose=[0, y_mm_gripper_width, 0, 0, 0, 0], relative=True, wait=True)
        # time.sleep(10)
        
        print(f" 9. move out X 9/10")
        self.arm.set_position_aa(axis_angle_pose=[self.reverse_x, 0, 0, 0, 0, 0], relative=True, wait=True)
        # time.sleep(10)
        
        print(f" 10. move out Y 10/10")
        self.arm.set_position_aa(axis_angle_pose=[0, self.reverse_y, 0, 0, 0, 0], relative=True, wait=True)
        # time.sleep(10)

        self.cur_status = "Unhooked"
        self.flag = 0

        return unhookResponse(status="Unhooked")

    @classmethod
    def arc_motion(self, req: arc_moveResponse) -> arc_moveRequest:
        # if VERBOSE:
        rospy.loginfo("Performing the arc motion")

        # check condition to verify if xArm should move through ext. mechanisms
        if(self.flag == 1):
            self.go2EM_plane()

        insertion_angle = req.move_angle
        self.angle += req.move_angle
        print("total angle: ", self.angle)

        tfBuffer = tf2_ros.Buffer(rospy.Duration(3.0))
        tf2_ros.TransformListener(tfBuffer)
        tf_lookup_done = False

        while not tf_lookup_done:
            try:
                trans = tfBuffer.lookup_transform('link_base', 'gripper', rospy.Time(), rospy.Duration(3.0))
                tf_lookup_done = True
                print(f" ******** transform_lookup TF lookup done ******** ")
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            # except Exception as e:
            # except (tf2_ros.LookupException):
                print(f" ******** transform_lookup TF lookup error ******** ")
                tf_lookup_done = False

        x_curr, y_curr = trans.transform.translation.x, trans.transform.translation.y
        
        # #change passed in values based on how API returns position
        x_pos, y_pos = self.calculate_position([x_curr, y_curr-0.15], 0.15, insertion_angle)

        print(insertion_angle)
        print("x,y: ", x_curr, y_curr)
        print("x,y: ", x_pos, y_pos)

        self.arm.set_position_aa(axis_angle_pose=[(x_pos-x_curr) * 1000, (y_pos-y_curr) * 1000, 0, 0, 0, -insertion_angle], speed=50, relative=True, wait=True, is_radian=False)
        print("DONE")

        # '''
        # rotation
        # '''
        # self.arm.set_position_aa(axis_angle_pose=[0, 0, 0, 0, 0, -insertion_angle], speed=50, relative=True, wait=True)

        # self.cur_status = "arc motion"
        # self.flag = 0

        return arc_moveResponse(angle=self.angle)

        

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