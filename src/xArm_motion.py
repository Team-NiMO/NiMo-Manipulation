#!/usr/bin/env python3
import numpy as np
import rospy
import time
import yaml
import rospkg
import tf2_ros
from xarm.wrapper import XArmAPI
from geometry_msgs.msg import Point, TransformStamped
from sensor_msgs.msg import JointState
from nimo_manipulation.srv import *

class xArm_Motion():
    @classmethod
    def __init__(self, ip_addr):
        self.loadConfig()

        if self.verbose: rospy.loginfo('Starting xArm_motion node.')
        
        try:
            rospy.wait_for_message("/xarm/joint_states", JointState, timeout=5)
        except rospy.ROSException:
            rospy.logwarn('Unable to connect to xArm')

        # Initialize xArm
        self.arm = XArmAPI(ip_addr)
        self.arm.motion_enable(enable=True)
        self.arm.set_mode(0)
        self.arm.set_state(state=0)

        # Set self collision for the xArm
        self.arm.set_self_collision_detection(on_off=True)
        self.arm.set_collision_tool_model(22, x=226.06, y=76.2, z=195.58)

        # Setup services
        self.get_xArm_service = rospy.Service('GoHome', GoHome, self.GoHome)
        self.get_xArm_service = rospy.Service('GoStow', GoStow, self.GoStow)
        self.get_xArm_service = rospy.Service('GoScan', GoScan, self.GoScan)
        self.get_xArm_service = rospy.Service('LookatCorn', LookatCorn, self.LookatCorn)
        self.get_xArm_service = rospy.Service('LookatAngle', LookatAngle, self.LookatAngle)
        self.get_xArm_service = rospy.Service('GoEM', GoEM, self.GoEM)
        self.get_xArm_service = rospy.Service('GoRM', GoRM, self.GoRM)
        self.get_xArm_service = rospy.Service('GoCorn', GoCorn, self.GoCorn)
        self.get_xArm_service = rospy.Service('UngoCorn', UngoCorn, self.UngoCorn)
        self.get_xArm_service = rospy.Service('ArcCorn', ArcCorn, self.ArcCorn)
        self.get_xArm_service = rospy.Service('HookCorn', HookCorn, self.HookCorn)
        self.get_xArm_service = rospy.Service('UnhookCorn', UnhookCorn, self.UnhookCorn)

        # Internal variables
        self.state = "STOW" # TODO: This may not be true on startup
        self.absolute_angle = 0 # angle at which the xarm is facing the cornstalk

        self.broadcaster = tf2_ros.StaticTransformBroadcaster()

        tfBuffer = tf2_ros.Buffer(rospy.Duration(3.0))
        tf2_ros.TransformListener(tfBuffer)

        if self.verbose: rospy.loginfo('Waiting for service calls...')        

    @classmethod
    def loadConfig(self):
        '''
        Load configuration from yaml file
        '''

        rospack = rospkg.RosPack()
        self.package_path = rospack.get_path('nimo_manipulation')
        config_path = self.package_path + '/config/default.yaml'
        with open(config_path) as file:
            config = yaml.load(file, Loader=yaml.FullLoader)

        self.verbose = config["debug"]["verbose"]
        self.approach = config["gripper"]["approach"]
        self.ip_address = config["arm"]["ip_address"]

    @classmethod 
    def GoScan(self, req: GoScanRequest) -> GoScanResponse:
        '''
        Move the xArm to the scan position (for detecting corn)
        
        Returns:
            GoScanResponse: The response:
                           - success - The success of the operation (DONE / ERROR)
        '''
        rospy.loginfo("Current State: {}".format(self.state))
        
        if self.verbose: rospy.loginfo('Going to Scan Position')

        # Joint angles corresponding to end-effector facing the left side of the amiga base
        code = self.arm.set_servo_angle(angle=[-90, -100, -40, 0, 50, -90], speed=30, is_radian=False, wait=True)

        if code != 0:
            rospy.logerr("set_servo_angle returned error {}".format(code))
            return GoStowResponse(success="ERROR")

        self.state = "SCAN"

        return GoScanResponse(success="DONE")


    @classmethod 
    def GoStow(self, req: GoStowRequest) -> GoStowResponse:
        '''
        Move the xArm to the stow position (for navigation)
        
        Returns:
            GoStowResponse: The response:
                           - success - The success of the operation (DONE / ERROR)
        '''
        rospy.loginfo("Current State: {}".format(self.state))
        
        if self.verbose: rospy.loginfo('Going to Stow Position')

        # Joint angles corresponding to end-effector facing the left side of the amiga base
        code = self.arm.set_servo_angle(angle=[0, -100, 5, 0, 5, -90], speed=30, is_radian=False, wait=True)

        if code != 0:
            rospy.logerr("set_servo_angle returned error {}".format(code))
            return GoStowResponse(success="ERROR")

        self.state = "STOW"

        return GoStowResponse(success="DONE")

    @classmethod 
    def GoHome(self, req: GoHomeRequest) -> GoHomeResponse:
        '''
        Move the xArm to the home position
        
        Returns:
            GoHomeResponse: The response:
                           - success - The success of the operation (DONE / ERROR)
        '''
        rospy.loginfo("Current State: {}".format(self.state))
        if self.state == "CORN_HOOK" or self.state == "CORN_OFFSET":
            rospy.logerr("Invalid Command: Cannot move from {} to {} via GoHome".format(self.state, "HOME"))
            return GoHomeResponse(success="ERROR")
        elif self.state in ["clean", "cal_low", "cal_high"]:
            self.GoEMPlane(-1)
        elif self.state in ["RM1", "RM2", "RM3", "RM4", "RM5"]:
            rospy.loginfo("Moving from {} to {} via GoHome".format(self.state, "HOME"))
            slot_val = int(self.state[2:3])
            rospy.loginfo('Starting xArm_motion node.')
            self.GoRMPrePos(slot_val, -1)

        if self.verbose: rospy.loginfo('Going to Home Position')

        # Joint angles corresponding to end-effector facing the left side of the amiga base
        code = self.arm.set_servo_angle(angle=[0, -90, 0, -90, 90, 0], speed=30, is_radian=False, wait=True)

        if code != 0:
            rospy.logerr("set_servo_angle returned error {}".format(code))
            return GoHomeResponse(success="ERROR")

        self.state = "HOME"

        return GoHomeResponse(success="DONE")
    
    @classmethod
    def LookatCorn(self, req: LookatCornRequest) -> LookatCornResponse:
        '''
        Moves the xArm to look at corn row for StalkDetect

        Returns:
            LookatCornResponse: The response
                                - success - The success of the operation (DONE / ERROR)
        '''

        if self.state != "HOME":
            rospy.logerr("Invalid Command: Cannot move from {} to LookatCorn".format(self.state))
            return LookatCornResponse(success="ERROR")
        elif self.state in ["clean", "cal_low", "cal_high"]:
            self.GoEMPlane(-1)

        if self.verbose: rospy.loginfo('Going to LookatCorn Position')

        # Joint angles corresponding to end-effector facing the left side of the amiga base
        code = self.arm.set_servo_angle(angle=[0, -90, 0, -115, 90, 0], is_radian=False, wait=True)

        if code != 0:
            rospy.logerr("set_servo_angle returned error {}".format(code))
            return LookatCornResponse(success="ERROR")

        self.state = "LookatCorn"

        return LookatCornResponse(success="DONE")
    
    @classmethod
    def LookatAngle(self, req: LookatAngleRequest) -> LookatAngleResponse:
        '''
        Moves the xArm to look at corn row at an angle for StalkDetect

        Returns:
            LookatAngleResponse: The response
                                - success - The success of the operation (DONE / ERROR)
        '''

        if self.state != "SCAN" and self.state != "LookatAngle":
            rospy.logerr("Invalid Command: Cannot move from {} to LookatAngle".format(self.state))
            return LookatAngleResponse(success="ERROR")
        elif self.state in ["clean", "cal_low", "cal_high"]:
            self.GoEMPlane()

        if self.verbose: rospy.loginfo('Going to LookatAngle Position')

        # Moving Joint-1 relative to the Scan position
        # to the left -> greater than 90; to the right -> lesser than 90
        code = self.arm.set_servo_angle(angle=[-90-req.joint_angle, -100, -40, 0, 50, -90], speed=30, is_radian=False, wait=True)
        # code = self.arm.set_servo_angle(angle=[0, -90, 0, -115, 90-req.joint_angle, 0], is_radian=False, wait=True)

        if code != 0:
            rospy.logerr("set_servo_angle returned error {}".format(code))
            return LookatAngleResponse(success="ERROR")

        self.state = "LookatAngle"

        return LookatAngleResponse(success="DONE")


    @classmethod
    def GoEMPlane(self, direction):
        '''
        Move the xArm to the external mechanisms plane
        '''
        
        if self.verbose: rospy.loginfo('Going to External Mechanisms Plane')
        if direction == 1:
        # Joint angles corresponding to external mechanisms plane
        # code = self.arm.set_servo_angle(angle=[-90, 41.5, -40.3, 0, -88.3, -90], is_radian=False, wait=True)

        # Joint angle for Janice's gripper
            code = self.arm.set_servo_angle(angle=[0, -10, 0, -90, 90, 0], is_radian=False, wait=True)
        # joint angle for the arbitary EM plane position (in front and under the EM)
            code = self.arm.set_servo_angle(angle=[-118.2, 36.1, -25.4, -116.9, 100.5, 29.5], is_radian=False, wait=True)
        
        else:
            code = self.arm.set_servo_angle(angle=[-118.2, 36.1, -25.4, -116.9, 100.5, 29.5], is_radian=False, wait=True)
            code = self.arm.set_servo_angle(angle=[0, -10, 0, -90, 90, 0], is_radian=False, wait=True)


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
                          - success - The success of the operation (DONE / ERROR)
        '''
        
        if req.id not in ["clean", "cal_low", "cal_high"]:
            rospy.logerr("Invalid Command: No such nozzle {}".format(req.id))
            return GoEMResponse(success="ERROR")
        
        if self.state == "SCAN":
            self.GoHome()

        if self.state == "HOME":
            self.GoEMPlane(1)
        # elif self.state != "EM":
        elif self.state not in ["clean", "cal_low", "cal_high"]:
            rospy.logerr("Invalid Command: Cannot move from {} to {}".format(self.state, "EM"))
            return GoEMResponse(success="ERROR")

        if req.id == "clean":
            if self.verbose: rospy.loginfo("Going to Cleaning Nozzle")

            if self.state == "cal_high":
                # move to the cal_low nozzle before moving to the clean nozzle
                # code = self.arm.set_servo_angle(angle=[-115.3, 89.1, -96, 64.4, -87.6, -102.1], is_radian=False, wait=True)

                # Janice's gripper cal_low nozzle joint value
                # code = self.arm.set_servo_angle(angle=[-127.3, 40.8, -46.9, -127.6, 91.8, 15.1], is_radian=False, wait=True)
                # new joint angle for janice's gripper on amiga
                code = self.arm.set_servo_angle(angle=[-125.2, 40.3, -46.9, -125.5, 91.7, 14.6], is_radian=False, wait=True)

            # Joint angles corresponding to end-effector at the cleaning nozzle
            # code = self.arm.set_servo_angle(angle=[-125.1, 85, -74.8, 55.2, -96.5, -87.3], is_radian=False, wait=True)
            # code = self.arm.set_servo_angle(angle=[-137.9, 37.3, -34.9, -137.7, 97.3, 21.6], is_radian=False, wait=True) # Janice's gripper
            # new joint angle for janice's gripper on amiga
            code = self.arm.set_servo_angle(angle=[-136.8, 36.5, -34.1, -136.6, 97.3, 21.6], is_radian=False, wait=True)

        elif req.id == "cal_low":
            if self.verbose: rospy.loginfo("Going to Low Calibration Nozzle")

            # Joint angles corresponding to end-effector at the low calibration nozzle
            # code = self.arm.set_servo_angle(angle=[-115.3, 89.1, -96, 64.4, -87.6, -102.1], is_radian=False, wait=True)
            # code = self.arm.set_servo_angle(angle=[-127.3, 40.8, -46.9, -127.6, 91.8, 15.1], is_radian=False, wait=True)
            # new joint angle for janice's gripper on amiga
            code = self.arm.set_servo_angle(angle=[-125.2, 40.3, -46.9, -125.5, 91.7, 14.6], is_radian=False, wait=True)


        elif req.id == "cal_high":
            if self.verbose: rospy.loginfo("Going to High Calibration Nozzle")

            if self.state == "clean":
                # move to the cal_low nozzle before moving to the cal_high nozzle
                # code = self.arm.set_servo_angle(angle=[-115.3, 89.1, -96, 64.4, -87.6, -102.1], is_radian=False, wait=True)
                # code = self.arm.set_servo_angle(angle=[-127.3, 40.8, -46.9, -127.6, 91.8, 15.1], is_radian=False, wait=True)
                # new joint angle for janice's gripper on amiga
                code = self.arm.set_servo_angle(angle=[-125.2, 40.3, -46.9, -125.5, 91.7, 14.6], is_radian=False, wait=True)

            
            # Joint angles corresponding to end-effector at the high calibration nozzle
            # code = self.arm.set_servo_angle(angle=[-108.7, 110.3, -149.4, 73, -76.8, -129.8], is_radian=False, wait=True)
            # code = self.arm.set_servo_angle(angle=[-119.9, 47.2, -63.2, -120.2, 87.4, 6.1], is_radian=False, wait=True) # Janice's gripper
            # new joint angle for janice's gripper on amiga
            code = self.arm.set_servo_angle(angle=[-117.8, 46.8, -63.1, -118.1, 87.7, 5.5], is_radian=False, wait=True)


        if code != 0:
            rospy.logerr("set_servo_angle returned error {}".format(code))
            return GoEMResponse(success="ERROR")
        
        self.state = req.id
        return GoEMResponse(success="DONE")


    @classmethod
    def GoRMPrePos(self, slot, direction):
        '''
        Move the xArm to the replacement mechanisms plane
        '''
        
        if self.verbose: rospy.loginfo("Going to Replacement Mechanisms Preload Position for Slot {}".format(slot))
        if direction == 1:
            code = self.arm.set_servo_angle(angle=[-180, -90, 0, -90, 90, 0], is_radian=False, wait=True)
            code = self.arm.set_servo_angle(angle=[-180, 0, 0, -90, 90, 90], is_radian=False, wait=True)

            if slot == 1:
                code = self.arm.set_servo_angle(angle=[-244.6, 45.8, -39.2, -244.8, 92.8, 175], is_radian=False, wait=True)
            elif slot == 2:
                code = self.arm.set_servo_angle(angle=[-246.7, 46.5, -43.8, -246.7, 91.1, 178.5], is_radian=False, wait=True)
            elif slot == 3:
                code = self.arm.set_servo_angle(angle=[-248.6, 47.6, -48.6, -248.6, 89.6, 181.9], is_radian=False, wait=True)
            elif slot == 4:
                code = self.arm.set_servo_angle(angle=[-249.7, 49.0, -53.7, -249.9, 88, 185.2], is_radian=False, wait=True)
            else:
                code = self.arm.set_servo_angle(angle=[-251.1, 50.8, -59.1, -251.4, 86.9, 188.7], is_radian=False, wait=True)
        
        else: ## direction == -1
            if slot == 1:
                code = self.arm.set_servo_angle(angle=[-244.6, 45.8, -39.2, -244.8, 92.8, 175], speed=10, is_radian=False, wait=True)
            elif slot == 2:
                code = self.arm.set_servo_angle(angle=[-246.7, 46.5, -43.8, -246.7, 91.1, 178.5], speed=10, is_radian=False, wait=True)
            elif slot == 3:
                code = self.arm.set_servo_angle(angle=[-248.6, 47.6, -48.6, -248.6, 89.6, 181.9], speed=10, is_radian=False, wait=True)
            elif slot == 4:
                code = self.arm.set_servo_angle(angle=[-249.7, 49.0, -53.7, -249.9, 88, 185.2], speed=10, is_radian=False, wait=True)
            else:
                code = self.arm.set_servo_angle(angle=[-251.1, 50.8, -59.1, -251.4, 86.9, 188.7], speed=10, is_radian=False, wait=True)

            code = self.arm.set_servo_angle(angle=[-180, 0, 0, -90, 90, 90], is_radian=False, wait=True)
            code = self.arm.set_servo_angle(angle=[-180, -90, 0, -90, 90, 0], is_radian=False, wait=True)

        if code != 0:
            rospy.logerr("set_servo_angle returned error {}".format(code))
        return code

    @classmethod
    def GoRM(self, req: GoRMRequest) -> GoRMResponse:
        '''
        Move the xArm to the replacement mechanisms at the specific nozzle

        Parameters:
            req (GoRMRequest): The request:
                               - id - which sensor slot to move to
        
        Returns:
            GoRMResponse: The response:
                          - success - The success of the operation (DONE / ERROR)
        '''
        
        if req.id not in [1, 2, 3, 4, 5]:
            rospy.logerr("Invalid Command: No such slot {}".format(req.id))
            return GoRMResponse(success="ERROR")

        if self.state == "HOME":
            slot_val = int(req.id)
            code = self.GoRMPrePos(slot_val, 1)
            if slot_val == 1:
                self.state = "RM1"
                code = self.arm.set_servo_angle(angle=[-227.2, 47.4, -48.7, -227.3, 89.1, 182], speed = 10, is_radian=False, wait=True)
                # code = self.GoRMPrePos(slot_val, -1)
            elif slot_val == 2:
                self.state = "RM2"
                code = self.arm.set_servo_angle(angle=[-230.4, 48.9, -53.1, -230.5, 87.3, 184.3], speed=10, is_radian=False, wait=True)
                # code = self.GoRMPrePos(slot_val, -1)
            elif slot_val == 3:
                self.state = "RM3"
                code = self.arm.set_servo_angle(angle=[-233.2, 50.6, -58  , -233.4, 85.5, 187], speed=10, is_radian=False, wait=True)
                # code = self.GoRMPrePos(slot_val, -1)
            elif slot_val == 4:
                self.state = "RM4"
                code = self.arm.set_servo_angle(angle=[-235.4, 52.1, -62.8, -236  , 83.5, 189.7], speed=10, is_radian=False, wait=True)
                # code = self.GoRMPrePos(slot_val, -1)
            else:
                self.state = "RM5"
                code = self.arm.set_servo_angle(angle=[-237.6, 54.1, -68.2, -238.5, 82.1, 192.7], speed=10, is_radian=False, wait=True)
                # code = self.GoRMPrePos(slot_val, -1)

            # slot_num = int(self.state[2:3])
            # rospy.loginfo("slot_num:{}, {}".format(slot_num, type(slot_num)))
            
        else:
            rospy.logerr("Invalid Command: Cannot move from {} to {}".format(self.state, "RM"))
            return GoRMResponse(success="ERROR")

        if code != 0:
            rospy.logerr("set_servo_angle returned error {}".format(code))
            return GoRMResponse(success="ERROR")
        
        # self.state = req.id
        return GoRMResponse(success="DONE")

    @classmethod
    def GoCorn(self, req: GoCornRequest) -> GoCornResponse:
        '''
        Move the xArm 15 cm away from the specified grasp point

        Parameters:
            req (GoCornRequest): The request:
                                 - grasp_point - the position to grasp the cornstalk in the world frame (m)
        
        Returns:
            GoCornResponse: The response:
                          - success - The success of the operation (DONE / ERROR)
        '''
        rospy.loginfo(self.state)
        #if self.state != "HOME":
        if self.state != "SCAN":
            rospy.logerr("Invalid Command: Cannot move from {} to {}".format(self.state, "CORN_OFFSET"))
            return GoCornResponse(success="ERROR")

        if self.verbose: rospy.loginfo("Going to the cornstalk {}, {}, {}".format(req.grasp_point.x, req.grasp_point.y, req.grasp_point.z))

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
        
        tfBuffer = tf2_ros.Buffer(rospy.Duration(3.0))

        # if req.grasp_point.x < 0.2:
        #     # include pre grasp pose
        #     # code = self.arm.set_servo_angle(angle=[-90, -90, 0, -90, 0, 0], is_radian=False, wait=True)

        #     # code = self.arm.set_servo_angle(angle=[0, -90, 0, -90, 90, 0], speed=30, is_radian=False, wait=True)

        #     if code != 0:
        #         rospy.logerr("set_arm_position_aa returned error {}".format(code))
        #         return GoCornResponse(success="ERROR")

        #     code = self.arm.set_position_aa(axis_angle_pose=[0, -5, 120, 0, 0, 0], speed=30, relative=True, wait=True)
        #     #120mm in z from home position
        #     # code = self.arm.set_servo_angle(angle=[-79.4, -81.1, -15.2, -59.5, 12.4, -31.1], is_radian=False, wait=True)

        #     if code != 0:
        #         rospy.logerr("set_arm_position_aa returned error {}".format(code))
        #         return GoCornResponse(success="ERROR")
            
        # sys.exit()
        
        # Get the relative movement to the cornstalk
        tf2_ros.TransformListener(tfBuffer)
        delta = tfBuffer.lookup_transform('corn_cam', 'gripper', rospy.Time(), rospy.Duration(3.0)).transform.translation
        # del_x, del_y, del_z = -delta.x * 1000, (-delta.y + 0.15) * 1000, -delta.z * 1000 
        del_x, del_y, del_z = -delta.x * 1000, (-delta.y + 0.08) * 1000, -delta.z * 1000 

        # Update relative movement with offset
        self.del_x, self.del_y, self.del_z = del_x, del_y, del_z

        code = self.arm.set_position_aa(axis_angle_pose=[0, 0, del_z, 0, 0, 0], speed=30, relative=True, wait=True)
        code = self.arm.set_position_aa(axis_angle_pose=[del_x, 0, 0, 0, 0, 0], speed=30, relative=True, wait=True)
        code = self.arm.set_position_aa(axis_angle_pose=[0, del_y, 0, 0, 0, 0], speed=30, relative=True, wait=True)
        

        if code != 0:
            rospy.logerr("set_arm_position_aa returned error {}".format(code))
            return GoCornResponse(success="ERROR")

        self.state = "CORN_OFFSET"
        return GoCornResponse(success="DONE")
    
    @classmethod
    def UngoCorn(self, req: UngoCornRequest) -> UngoCornResponse:
        '''
        Reverse motion of GoCorn and return to home position
        
        Returns:
            UngoCornResponse: The response:
                           - success - The success of the operation (DONE / ERROR)
        '''
        if self.state != "CORN_OFFSET":
            rospy.logerr("Invalid Command: Cannot move from {} to {} via UngoCorn".format(self.state, "HOME"))
            return UngoCornResponse(success="ERROR")

        if self.verbose: rospy.loginfo("Unapproaching the cornstalk")
        code = self.arm.set_position_aa(axis_angle_pose=[0, -self.del_y, 0, 0, 0, 0], speed=30, relative=True, wait=True)
        code = self.arm.set_position_aa(axis_angle_pose=[-self.del_x, 0, 0, 0, 0, 0], speed=30, relative=True, wait=True)
        code = self.arm.set_position_aa(axis_angle_pose=[0, 0, -self.del_z, 0, 0, 0], speed=30, relative=True, wait=True)
        
        # code = self.arm.set_position_aa(axis_angle_pose=[-self.del_x, -self.del_y, -self.del_z, 0, 0, 0], speed=30, relative=True, wait=True)

        if code != 0:
            rospy.logerr("set_arm_position_aa returned error {}".format(code))
            return UngoCornResponse(success="ERROR")
        
        # Back to Scan
        # code = self.arm.set_servo_angle(angle=[0, -90, 0, -90, 90, 0], speed=30, is_radian=False, wait=True)
        code = self.arm.set_servo_angle(angle=[-90, -100, -40, 0, 50, -90], speed=30, is_radian=False, wait=True)
        if code != 0:
            rospy.logerr("set_arm_position_aa returned error {}".format(code))
            return UngoCornResponse(success="ERROR")


        self.state = "SCAN"
        return UngoCornResponse(success="DONE")

    @classmethod
    def HookCorn(self, req: HookCornRequest) -> HookCornResponse:
        '''
        Hook the corn at the specified grasp point using the end effector

        Parameters:
            req (HookCornRequest): The request:
                                  - grasp_point - the position to grasp the cornstalk in the world frame (m)
        
        Returns:
            HookCornResponse: The response:
                              - success - The success of the operation (DONE / ERROR)
        '''

        if self.state != "SCAN":
            rospy.logerr("Invalid Command: Cannot move from {} to {}".format(self.state, "CORN_HOOK"))
            return HookCornResponse(success="ERROR")

        if self.verbose: rospy.loginfo("Hooking cornstalk {}, {}, {}".format(req.grasp_point.x, req.grasp_point.y, req.grasp_point.z))

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

        tfBuffer = tf2_ros.Buffer(rospy.Duration(3.0))
        tf2_ros.TransformListener(tfBuffer)

        # if req.grasp_point.x < 0.2:
        #     # include pre grasp pose
        #     # code = self.arm.set_servo_angle(angle=[-90, -90, 0, -90, 0, 0], is_radian=False, wait=True)

        #     code = self.arm.set_servo_angle(angle=[0, -90, 0, -90, 90, 0], speed=30, is_radian=False, wait=True)
        #     if code != 0:
        #         rospy.logerr("set_servo_angle returned error {}".format(code))
        #         return HookCornResponse(success="ERROR")
            
            # code = self.arm.set_servo_angle(angle=[-79.4, -81.1, -15.2, -59.5, 12.4, -31.1], speed=30, is_radian=False, wait=True)

            # code = self.arm.set_position_aa(axis_angle_pose=[0, -5, 120, 0, 0, 0], speed=30, relative=True, wait=True)

            # if code != 0:
            #     rospy.logerr("set_servo_angle returned error {}".format(code))
            #     return HookCornResponse(success="ERROR")

        # Get the relative movement to the cornstalk
        delta = tfBuffer.lookup_transform('corn_cam', 'gripper', rospy.Time(), rospy.Duration(3.0)).transform.translation
        del_x, del_y, del_z = -delta.x * 1000, -delta.y * 1000, -delta.z * 1000 
        del_x += (del_x - 0.144) * 0.026 + 0.05
        del_y -= 10

        if self.approach == "left":
            # Move to pre-grasp 1/2
            x_mov, y_mov, z_mov = del_x-85, del_y+150, del_z
            
            # stored for unhook "pre-grasp"
            
            # MODIFIED
            self.x_mov_unhook, self.y_mov_unhook, self.z_mov_unhook = -x_mov, -y_mov, -z_mov
            # code = self.arm.set_position_aa(axis_angle_pose=[x_mov, y_mov, z_mov, 0, 0, 0], speed=50, relative=True, wait=True)
            code = self.arm.set_position_aa(axis_angle_pose=[0, y_mov, z_mov, 0, 0, 0], speed=50, relative=True, wait=True)
            code = self.arm.set_position_aa(axis_angle_pose=[x_mov, 0, 0, 0, 0, 0], speed=50, relative=True, wait=True)

            if code != 0:
                rospy.logerr("set_arm_position_aa returned error {}".format(code))
                return HookCornResponse(success="ERROR")
            
            # Move to pre-grasp 2/2
            code = self.arm.set_position_aa(axis_angle_pose=[0, -150, 0, 0, 0, 0], speed=30, relative=True, wait=True)

            if code != 0:
                rospy.logerr("set_arm_position_aa returned error {}".format(code))
                return HookCornResponse(success="ERROR")
            
            # Move to grasp
            code = self.arm.set_position_aa(axis_angle_pose=[85, 0, 0, 0, 0, 0], speed=30, relative=True, wait=True)

            if code != 0:
                rospy.logerr("set_arm_position_aa returned error {}".format(code))
                return HookCornResponse(success="ERROR")
        
        elif self.approach == "front":
            x_mov, y_mov, z_mov = del_x, del_y+80, del_z
            self.x_mov_unhook, self.y_mov_unhook, self.z_mov_unhook = -x_mov, -y_mov, -z_mov
            code = self.arm.set_position_aa(axis_angle_pose=[0, 0, z_mov, 0, 0, 0], speed=30, relative=True, wait=True)
            code = self.arm.set_position_aa(axis_angle_pose=[x_mov, 0, 0, 0, 0, 0], speed=30, relative=True, wait=True)

            if code != 0:
                rospy.logerr("set_arm_position_aa returned error {}".format(code))
                return HookCornResponse(success="ERROR")
            
            code = self.arm.set_position_aa(axis_angle_pose=[0, y_mov, 0, 0, 0, 0], speed=50, relative=True, wait=True)

            if code != 0:
                rospy.logerr("set_arm_position_aa returned error {}".format(code))
                return HookCornResponse(success="ERROR")
            
            code = self.arm.set_position_aa(axis_angle_pose=[0, -80, 0, 0, 0, 0], speed=30, relative=True, wait=True)

            if code != 0:
                rospy.logerr("set_arm_position_aa returned error {}".format(code))
                return HookCornResponse(success="ERROR")

        # Move to insertion angle

        trans = tfBuffer.lookup_transform('link_base', 'gripper', rospy.Time(), rospy.Duration(3.0)).transform.translation
        radius = tfBuffer.lookup_transform('link_eef', 'gripper', rospy.Time(), rospy.Duration(3.0)).transform.translation.z
        x_curr, y_curr = trans.x, trans.y

        # Determine the offset to move in x and y 
        x_pos = x_curr + radius * np.sin(np.radians(req.insert_angle))
        y_pos = (y_curr-radius) + radius * np.cos(np.radians(req.insert_angle))
        del_x, del_y = (x_pos-x_curr) * 1000, (y_pos-y_curr) * 1000

        self.unhook_x, self.unhook_y = -del_x, -del_y

        self.absolute_angle = -req.insert_angle
        code = self.arm.set_position_aa(axis_angle_pose=[del_x, del_y, 0, 0, 0, self.absolute_angle], speed=30, relative=True, wait=True, is_radian=False)
        
        if code != 0:
            rospy.logerr("set_arm_position_aa returned error {}".format(code))
            return HookCornResponse(success="ERROR")

        self.state = "CORN_HOOK"
        return HookCornResponse(success="DONE")

    @classmethod
    def UnhookCorn(self, req: UnhookCornRequest) -> UnhookCornResponse:
        '''
        Unhook the cornstalk and return to home position
        
        Returns:
            UnhookCornResponse: The response:
                           - success - The success of the operation (DONE / ERROR)
        '''
        
        if self.state != "CORN_HOOK":
            rospy.logerr("Invalid Command: Cannot move from {} to {} via UnhookCorn".format(self.state, "HOME"))
            return UnhookCornResponse(success="ERROR")

        if self.verbose: rospy.loginfo("Unhooking cornstalk")

        code = self.arm.set_position_aa(axis_angle_pose=[self.unhook_x, self.unhook_y, 0, 0, 0, -self.absolute_angle], speed=30, relative=True, wait=True, is_radian=False)
        
        if code != 0:
            rospy.logerr("set_arm_position_aa returned error {}".format(code))
            return UnhookCornResponse(success="ERROR")

        if self.approach == "left":
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
            
        elif self.approach == "front":
            # Move to pre-grasp
            code = self.arm.set_position_aa(axis_angle_pose=[0, 80, 0, 0, 0, 0], speed=30, relative=True, wait=True)

            if code != 0:
                rospy.logerr("set_arm_position_aa returned error {}".format(code))
                return UnhookCornResponse(success="ERROR")
        
        
        code = self.arm.set_position_aa(axis_angle_pose=[0, self.y_mov_unhook, 0, 0, 0, 0], speed=30, relative=True, wait=True)
        code = self.arm.set_position_aa(axis_angle_pose=[self.x_mov_unhook, 0, 0, 0, 0, 0], speed=30, relative=True, wait=True)
        code = self.arm.set_position_aa(axis_angle_pose=[0, 0, self.z_mov_unhook, 0, 0, 0], speed=30, relative=True, wait=True)

        if code != 0:
            rospy.logerr("set_arm_position_aa returned error {}".format(code))
            return UnhookCornResponse(success="ERROR")
        
        # Scan pos
        code = self.arm.set_servo_angle(angle=[-90, -100, -40, 0, 50, -90], speed=30, is_radian=False, wait=True)
        # Stow pos
        code = self.arm.set_servo_angle(angle=[0, -100, 5, 0, 5, -90], speed=30, is_radian=False, wait=True)



        if code != 0:
            rospy.logerr("set_servo_angle returned error {}".format(code))
            return UnhookCornResponse(success="ERROR")

        self.state = "STOW"
        return UnhookCornResponse(success="DONE")

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
                              - success - The success of the operation (DONE / ERROR)
        '''

        if self.state != "CORN_OFFSET" and self.state!="CORN_HOOK":
            rospy.logerr("Invalid Command: Cannot move from {} to {}".format(self.state, "CORN_OFFSET"))
            return ArcCornResponse(success="ERROR")

        if self.verbose: rospy.loginfo("Performing the arc motion")
        
        tfBuffer = tf2_ros.Buffer(rospy.Duration(3.0))
        tf2_ros.TransformListener(tfBuffer)

        # Get the current gripper position
        trans = tfBuffer.lookup_transform('link_base', 'link_eef', rospy.Time(), rospy.Duration(3.0)).transform.translation
        radius = tfBuffer.lookup_transform('link_eef', 'gripper', rospy.Time(), rospy.Duration(3.0)).transform.translation.z
        x_curr, y_curr = trans.x, trans.y

        c_x = x_curr - (0.1 + radius) * np.sin(np.radians(self.absolute_angle))
        c_y = y_curr - (0.1 + radius) * np.cos(np.radians(self.absolute_angle))

        # Determine the offset to move in x and y
        x_pos = c_x + (0.1 + radius) * np.sin(np.radians(req.relative_angle + self.absolute_angle))
        y_pos = c_y + (0.1 + radius) * np.cos(np.radians(req.relative_angle + self.absolute_angle))
        del_x, del_y = (x_pos-x_curr) * 1000, (y_pos-y_curr) * 1000

        # Move to offset w/ yaw angle
        code = self.arm.set_position_aa(axis_angle_pose=[del_x, del_y, 0, 0, 0, -req.relative_angle], speed=40, relative=True, wait=True, is_radian=False)
        
        # trans = tfBuffer.lookup_transform('link_base', 'gripper', rospy.Time(), rospy.Duration(3.0)).transform.translation
        # radius = tfBuffer.lookup_transform('link_eef', 'gripper', rospy.Time(), rospy.Duration(3.0)).transform.translation.z
        # x_curr, y_curr = trans.x, trans.y

        # # Determine the offset to move in x and y 
        # x_pos = x_curr + radius * np.sin(np.radians(req.relative_angle))
        # y_pos = (y_curr-radius) + radius * np.cos(np.radians(req.relative_angle))
        # del_x, del_y = (x_pos-x_curr) * 1000, (y_pos-y_curr) * 1000

        # self.unhook_x, self.unhook_y = -del_x, -del_y

        # self.absolute_angle = -req.relative_angle
        # code = self.arm.set_position_aa(axis_angle_pose=[del_x, del_y, 0, 0, 0, self.absolute_angle], speed=30, relative=True, wait=True, is_radian=False)

        # if code != 0:
        #     rospy.logerr("set_arm_position_aa returned error {}".format(code))
        #     return ArcCornResponse(success="ERROR")

        self.absolute_angle += req.relative_angle

        self.state = "CORN_OFFSET"
        return ArcCornResponse(absolute_angle=self.absolute_angle, success="DONE")
    

if __name__ == '__main__':
    rospy.init_node('nimo_manipulation')
    detect_node = xArm_Motion('192.168.1.196')
    rospy.spin()
