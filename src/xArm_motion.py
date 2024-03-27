#!/usr/bin/env python3
import numpy as np
import rospy
import time
import tf2_ros
from xarm.wrapper import XArmAPI
from geometry_msgs.msg import Point, TransformStamped

from stalk_detect.srv import GetStalk
from nimo_manipulation.srv import *

VERBOSE = True

class xArm_Motion():
    @classmethod
    def __init__(self, ip_addr):
        if VERBOSE: rospy.loginfo('Starting xArm_motion node.')
        
        # Initialize xArm
        self.arm = XArmAPI(ip_addr)
        self.arm.motion_enable(enable=True)
        self.arm.set_mode(0)
        self.arm.set_state(state=0)

        # Setup services
        self.get_xArm_service = rospy.Service('GoHome', GoHome, self.GoHome)
        self.get_xArm_service = rospy.Service('GoEM', GoEM, self.GoEM)
        self.get_xArm_service = rospy.Service('GoCorn', GoCorn, self.GoCorn)
        self.get_xArm_service = rospy.Service('ArcCorn', ArcCorn, self.ArcCorn)
        self.get_xArm_service = rospy.Service('HookCorn', HookCorn, self.HookCorn)
        self.get_xArm_service = rospy.Service('UnhookCorn', UnhookCorn, self.UnhookCorn)

        # TODO: Cleanup these variables
        self.cur_status = None
        self.flag = 0
        self.reverse_x = None # for unhooking 
        self.reverse_y = None # for unhooking
        self.absolute_angle = 0 # angle at which the xarm is facing the cornstalk

        self.broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.tfBuffer = tf2_ros.Buffer(rospy.Duration(3.0))
        tf2_ros.TransformListener(self.tfBuffer)

        if VERBOSE: rospy.loginfo('Waiting for service calls...')        

    @classmethod 
    def GoHome(self, req: GoHomeRequest) -> GoHomeResponse:
        '''
        Move the xArm to the home position
        
        Returns:
            GoHomeResponse: The response:
                           - success - The success of the operation (SUCCESS / ERROR)
        '''

        if VERBOSE: rospy.loginfo('Going to Home Position')

        # check condition to verify if xArm should move through the ext. mechanisms plane
        if(self.flag == 1):
            self.GoEMPlane()

        # Joint angles corresponding to end-effector facing the left side of the amiga base
        code = self.arm.set_servo_angle(angle=[0, -90, 0, -90, 90, 0], is_radian=False, wait=True)

        if code != 0:
            rospy.logerr("set_servo_angle returned error {}".format(code))
            return GoHomeResponse(success="ERROR")

        self.cur_status = "home"
        self.flag = 0

        return GoHomeResponse(success="SUCCESS")

    @classmethod
    def GoEMPlane(self):
        '''
        Move the xArm to the external mechanisms plane
        '''
        
        if VERBOSE: rospy.loginfo('Going to External Mechanisms Plane')

        # Joint angles corresponding to external mechanisms plane
        code = self.arm.set_servo_angle(angle=[-90, 41.5, -40.3, 0, -88.3, 0], is_radian=False, wait=True)

        if code != 0:
            rospy.logerr("set_servo_angle returned error {}".format(code))

    @classmethod
    def GoEM(self, req: GoEMRequest) -> GoEMResponse:
        '''
        Move the xArm to the external mechanisms at the specific nozzle

        Parameters:
            req (GoEMRequest): The request:
                               - id - which nozzle to move to
        
        Returns:
            GoEMResponse: The response:
                          - success - The success of the operation (SUCCESS / ERROR)
        '''
        
        # check condition to verify if xArm should move through ext. mechanisms
        if(self.flag == 0):
            self.GoEMPlane()

        if req.id == "clean":
            if VERBOSE: rospy.loginfo("Going to Cleaning Nozzle")

            # Joint angles corresponding to end-effector at the cleaning nozzle
            code = self.arm.set_servo_angle(angle=[-133.6, 56.9, -53.4, 46, -85.6, 0], is_radian=False, wait=True)

        elif req.id == "cal_low":
            if VERBOSE: rospy.loginfo("Going to Low Calibration Nozzle")

            # Joint angles corresponding to end-effector at the low calibration nozzle
            code = self.arm.set_servo_angle(angle=[-118.3, 60.2, -69.9, 62.8, -77.9, -13.2], is_radian=False, wait=True)

        elif req.id == "cal_high":
            if VERBOSE: rospy.loginfo("Going to High Calibration Nozzle")
            
            # Joint angles corresponding to end-effector at the high calibration nozzle
            code = self.arm.set_servo_angle(angle=[-110.4, 75.9, -107.8, 73, -74.5, -33.8], is_radian=False, wait=True)

        if code != 0:
            rospy.logerr("set_servo_angle returned error {}".format(code))
            return GoEMResponse(success="ERROR")

        self.cur_status = "EM"
        self.flag = 1

        return GoEMResponse(success="SUCCESS")

    @classmethod
    def GoCorn(self, req: GoCornRequest) -> GoCornResponse:
        '''
        Move the xArm 15 cm away from the specified grasp point

        Parameters:
            req (GoCornRequest): The request:
                                 - grasp_point - the position to grasp the cornstalk in the world frame (m)
        
        Returns:
            GoCornResponse: The response:
                          - success - The success of the operation (SUCCESS / ERROR)
        '''

        if VERBOSE: rospy.loginfo("Going to the cornstalk {}, {}, {}".format(req.grasp_point.x, req.grasp_point.y, req.grasp_point.z))

        # check condition to verify if xArm should move through ext. mechanisms
        if(self.flag == 1):
            self.GoEMPlane()

        # Reset the absolute angle to the cornstalk
        self.absolute_angle = 0

        # Create the transform to the grasp point
        cornTransform = TransformStamped()
        cornTransform.header.stamp = rospy.Time.now()
        cornTransform.header.frame_id = "world"
        cornTransform.child_frame_id = "corn_cam"
        
        cornTransform.transform.translation.x = req.grasp_point.x
        cornTransform.transform.translation.y = req.grasp_point.y
        cornTransform.transform.translation.z = req.grasp_point.z
        cornTransform.transform.rotation.w = 1.0

        self.broadcaster.sendTransform(cornTransform)
        
        # Get the relative movement to the cornstalk
        delta = self.tfBuffer.lookup_transform('corn_cam', 'gripper', rospy.Time(), rospy.Duration(3.0)).transform.translation
        del_x, del_y, del_z = -delta.x * 1000, -delta.y * 1000, -delta.z * 1000 

        # Update relative movement with offset
        del_x, del_y, del_z = del_x, del_y+150, del_z

        code = self.arm.set_position_aa(axis_angle_pose=[del_x, del_y, del_z, 0, 0, 0], speed=50, relative=True, wait=True)

        if code != 0:
            rospy.logerr("set_arm_position_aa returned error {}".format(code))
            return GoEMResponse(success="ERROR")

        self.cur_status = "moved to corn"

        return GoCornResponse(success="SUCCESS")
    
    @classmethod
    def calculateArcOffset(self, centre, radius, angle):
        '''
        Determine the desired offset position from the cornstalk given a yaw angle

        Intended to be used with global coordinates looking left on the Amiga

        Parameters:
            center - the center point to get an offset from
            radius - the radius of the offset
            angle - the angle of the offset
        
        Returns:
            offset_x, offset_y - the x and y positions of the offset.
        '''

        offset_x = centre[0] + radius * np.sin(np.radians(angle))
        offset_y = centre[1] + radius * np.cos(np.radians(angle))
        return offset_x, offset_y

    @classmethod
    def HookCorn(self, req: HookCornRequest) -> HookCornResponse:
        '''
        Hook the corn at the specified grasp point using the end effector

        Parameters:
            req (HookCornRequest): The request:
                                  - grasp_point - the position to grasp the cornstalk in the world frame (m)
        
        Returns:
            HookCornResponse: The response:
                              - success - The success of the operation (SUCCESS / ERROR)
        '''

        if VERBOSE: rospy.loginfo("Hooking cornstalk {}, {}, {}".format(req.grasp_point.x, req.grasp_point.y, req.grasp_point.z))

        # check condition to verify if xArm should move through ext. mechanisms
        if(self.flag == 1):
            self.GoEMPlane()

        # Create the transform to the grasp point
        cornTransform = TransformStamped()
        cornTransform.header.stamp = rospy.Time.now()
        cornTransform.header.frame_id = "world"
        cornTransform.child_frame_id = "corn_cam"
        
        cornTransform.transform.translation.x = req.grasp_point.x
        cornTransform.transform.translation.y = req.grasp_point.y
        cornTransform.transform.translation.z = req.grasp_point.z
        cornTransform.transform.rotation.w = 1.0

        self.broadcaster.sendTransform(cornTransform)

        # Get the relative movement to the cornstalk
        delta = self.tfBuffer.lookup_transform('corn_cam', 'gripper', rospy.Time(), rospy.Duration(3.0)).transform.translation
        del_x, del_y, del_z = -delta.x * 1000, -delta.y * 1000, -delta.z * 1000 

        # Move to pre-grasp 1/2
        x_mov, y_mov, z_mov = del_x-85, del_y+150, del_z
        code = self.arm.set_position_aa(axis_angle_pose=[x_mov, y_mov, z_mov, 0, 0, 0], speed=50, relative=True, wait=True)

        if code != 0:
            rospy.logerr("set_arm_position_aa returned error {}".format(code))
            return GoEMResponse(success="ERROR")
        
        # Move to pre-grasp 2/2
        code = self.arm.set_position_aa(axis_angle_pose=[0, -150, 0, 0, 0, 0], speed=50, relative=True, wait=True)

        if code != 0:
            rospy.logerr("set_arm_position_aa returned error {}".format(code))
            return GoEMResponse(success="ERROR")
        
        # Move to grasp
        code = self.arm.set_position_aa(axis_angle_pose=[85, 0, 0, 0, 0, 0], speed=50, relative=True, wait=True)

        if code != 0:
            rospy.logerr("set_arm_position_aa returned error {}".format(code))
            return GoEMResponse(success="ERROR")

        self.cur_status = "hooked"
        self.flag = 0

        return HookCornResponse(success="SUCCESS")

    @classmethod
    def UnhookCorn(self, req: UnhookCornRequest) -> UnhookCornResponse:
        '''
        Unhook the cornstalk and return to home position
        
        Returns:
            UnhookCornResponse: The response:
                           - success - The success of the operation (SUCCESS / ERROR)
        '''
        
        if VERBOSE: rospy.loginfo("Unhooking cornstalk")

        # Move to ungrasp
        code = self.arm.set_position_aa(axis_angle_pose=[-85, 0, 0, 0, 0, 0], speed=50, relative=True, wait=True)

        if code != 0:
            rospy.logerr("set_arm_position_aa returned error {}".format(code))
            return UnhookCornResponse(success="ERROR")
        
        # Move to pre-grasp
        code = self.arm.set_position_aa(axis_angle_pose=[0, 150, 0, 0, 0, 0], speed=50, relative=True, wait=True)

        if code != 0:
            rospy.logerr("set_arm_position_aa returned error {}".format(code))
            return UnhookCornResponse(success="ERROR")
        
        # Joint angles corresponding to end-effector facing the left side of the amiga base
        code = self.arm.set_servo_angle(angle=[0, -90, 0, -90, 90, 0], is_radian=False, wait=True)

        if code != 0:
            rospy.logerr("set_servo_angle returned error {}".format(code))
            return UnhookCornResponse(success="ERROR")

        return UnhookCornResponse(success="SUCCESS")

    @classmethod
    def ArcCorn(self, req: ArcCornRequest) -> ArcCornResponse:
        '''
        Arc around the cornstalk by the specified angle

        Parameters:
            req (ArcCornRequest): The request:
                                  - relative_angle - The relative angle to rotate about the cornstalk (deg)
        
        Returns:
            ArcCornResponse: The response:
                              - absolute_angle - The absolute angle of the current position
                              - success - The success of the operation (SUCCESS / ERROR)
        '''

        if VERBOSE: rospy.loginfo("Performing the arc motion")

        # check condition to verify if xArm should move through ext. mechanisms
        if(self.flag == 1):
            self.GoEMPlane()

        # Get the current gripper position
        trans = self.tfBuffer.lookup_transform('link_base', 'gripper', rospy.Time(), rospy.Duration(3.0)).transform.translation
        x_curr, y_curr = trans.x, trans.y
        
        # Determine the offset to move in x and y
        x_pos, y_pos = self.calculateArcOffset([x_curr, y_curr-0.15], 0.15, req.relative_angle)
        del_x, del_y = (x_pos-x_curr) * 1000, (y_pos-y_curr) * 1000

        # Move to offset w/ yaw angle
        code = self.arm.set_position_aa(axis_angle_pose=[del_x, del_y, 0, 0, 0, -req.relative_angle], speed=50, relative=True, wait=True, is_radian=False)
        
        if code != 0:
            rospy.logerr("set_arm_position_aa returned error {}".format(code))
            return ArcCornResponse(success="ERROR")

        self.absolute_angle += req.relative_angle

        return ArcCornResponse(absolute_angle=self.absolute_angle, success="SUCCESS")
    

if __name__ == '__main__':
    rospy.init_node('nimo_manipulation')
    detect_node = xArm_Motion('192.168.1.214')
    rospy.spin()