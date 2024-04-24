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
        self.get_xArm_service = rospy.Service('LookatCorn', LookatCorn, self.LookatCorn)
        self.get_xArm_service = rospy.Service('LookatAngle', LookatAngle, self.LookatAngle)
        self.get_xArm_service = rospy.Service('GoEM', GoEM, self.GoEM)
        self.get_xArm_service = rospy.Service('GoCorn', GoCorn, self.GoCorn)
        self.get_xArm_service = rospy.Service('UngoCorn', UngoCorn, self.UngoCorn)
        self.get_xArm_service = rospy.Service('ArcCorn', ArcCorn, self.ArcCorn)
        self.get_xArm_service = rospy.Service('HookCorn', HookCorn, self.HookCorn)
        self.get_xArm_service = rospy.Service('UnhookCorn', UnhookCorn, self.UnhookCorn)

        # Internal variables
        self.state = "HOME" # TODO: This may not be true on startup
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

    @classmethod 
    def GoHome(self, req: GoHomeRequest) -> GoHomeResponse:
        '''
        Move the xArm to the home position
        
        Returns:
            GoHomeResponse: The response:
                           - success - The success of the operation (DONE / ERROR)
        '''

        if self.state == "CORN_HOOK":
            rospy.logerr("Invalid Command: Cannot move from {} to {} via GoHome".format(self.state, "HOME"))
            return GoHomeResponse(success="ERROR")
        elif self.state in ["clean", "cal_low", "cal_high"]:
            self.GoEMPlane()

        if self.verbose: rospy.loginfo('Going to Home Position')

        # Joint angles corresponding to end-effector facing the left side of the amiga base
        code = self.arm.set_servo_angle(angle=[0, -90, 0, -90, 90, 0], is_radian=False, wait=True)

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
            self.GoEMPlane()

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

        if self.state != "LookatCorn" and self.state != "LookatAngle":
            rospy.logerr("Invalid Command: Cannot move from {} to LookatAngle".format(self.state))
            return LookatAngleResponse(success="ERROR")
        elif self.state in ["clean", "cal_low", "cal_high"]:
            self.GoEMPlane()

        if self.verbose: rospy.loginfo('Going to LookatAngle Position')

        # Moving Joint-5 relative to the LookatCorn position
        # to the left -> greater than 90; to the right -> lesser than 90
        code = self.arm.set_servo_angle(angle=[0, -90, 0, -115, 90-req.joint_angle, 0], is_radian=False, wait=True)

        if code != 0:
            rospy.logerr("set_servo_angle returned error {}".format(code))
            return LookatAngleResponse(success="ERROR")

        self.state = "LookatAngle"

        return LookatAngleResponse(success="DONE")


    @classmethod
    def GoEMPlane(self):
        '''
        Move the xArm to the external mechanisms plane
        '''
        
        if self.verbose: rospy.loginfo('Going to External Mechanisms Plane')

        # Joint angles corresponding to external mechanisms plane
        code = self.arm.set_servo_angle(angle=[-90, 41.5, -40.3, 0, -88.3, -90], is_radian=False, wait=True)

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

        if self.state == "HOME":
            self.GoEMPlane()
        # elif self.state != "EM":
        elif self.state not in ["clean", "cal_low", "cal_high"]:
            rospy.logerr("Invalid Command: Cannot move from {} to {}".format(self.state, "EM"))
            return GoEMResponse(success="ERROR")

        if req.id == "clean":
            if self.verbose: rospy.loginfo("Going to Cleaning Nozzle")

            if self.state == "cal_high":
                # move to the cal_low nozzle before moving to the clean nozzle
                code = self.arm.set_servo_angle(angle=[-115.3, 89.1, -96, 64.4, -87.6, -102.1], is_radian=False, wait=True)

            # Joint angles corresponding to end-effector at the cleaning nozzle
            code = self.arm.set_servo_angle(angle=[-125.1, 85, -74.8, 55.2, -96.5, -87.3], is_radian=False, wait=True)

        elif req.id == "cal_low":
            if self.verbose: rospy.loginfo("Going to Low Calibration Nozzle")

            # Joint angles corresponding to end-effector at the low calibration nozzle
            code = self.arm.set_servo_angle(angle=[-115.3, 89.1, -96, 64.4, -87.6, -102.1], is_radian=False, wait=True)

        elif req.id == "cal_high":
            if self.verbose: rospy.loginfo("Going to High Calibration Nozzle")

            if self.state == "clean":
                # move to the cal_low nozzle before moving to the cal_high nozzle
                code = self.arm.set_servo_angle(angle=[-115.3, 89.1, -96, 64.4, -87.6, -102.1], is_radian=False, wait=True)
            
            # Joint angles corresponding to end-effector at the high calibration nozzle
            code = self.arm.set_servo_angle(angle=[-108.7, 110.3, -149.4, 73, -76.8, -129.8], is_radian=False, wait=True)

        if code != 0:
            rospy.logerr("set_servo_angle returned error {}".format(code))
            return GoEMResponse(success="ERROR")
        
        self.state = req.id
        return GoEMResponse(success="DONE")

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

        if self.state != "HOME":
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

        if req.grasp_point.x < 0.2:
            # include pre grasp pose
            code = self.arm.set_servo_angle(angle=[-90, -90, 0, -90, 0, 0], is_radian=False, wait=True)

            if code != 0:
                rospy.logerr("set_servo_angle returned error {}".format(code))
                return GoHomeResponse(success="ERROR")
        
        # Get the relative movement to the cornstalk
        tf2_ros.TransformListener(tfBuffer)
        delta = tfBuffer.lookup_transform('corn_cam', 'gripper', rospy.Time(), rospy.Duration(3.0)).transform.translation
        # del_x, del_y, del_z = -delta.x * 1000, (-delta.y + 0.15) * 1000, -delta.z * 1000 
        del_x, del_y, del_z = -delta.x * 1000, (-delta.y + 0.1) * 1000, -delta.z * 1000 

        # Update relative movement with offset
        self.del_x, self.del_y, self.del_z = del_x, del_y, del_z

        code = self.arm.set_position_aa(axis_angle_pose=[del_x, del_y, del_z, 0, 0, 0], speed=50, relative=True, wait=True)

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

        code = self.arm.set_position_aa(axis_angle_pose=[-self.del_x, -self.del_y, -self.del_z, 0, 0, 0], speed=50, relative=True, wait=True)

        if code != 0:
            rospy.logerr("set_arm_position_aa returned error {}".format(code))
            return UngoCornResponse(success="ERROR")
        
        # Joint angles corresponding to end-effector facing the left side of the amiga base
        code = self.arm.set_servo_angle(angle=[0, -90, 0, -90, 90, 0], is_radian=False, wait=True)

        if code != 0:
            rospy.logerr("set_arm_position_aa returned error {}".format(code))
            return UngoCornResponse(success="ERROR")

        self.state = "HOME"
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

        if self.state != "HOME":
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

        if req.grasp_point.x < 0.2:
            # include pre grasp pose
            code = self.arm.set_servo_angle(angle=[-90, -90, 0, -90, 0, 0], is_radian=False, wait=True)

            if code != 0:
                rospy.logerr("set_servo_angle returned error {}".format(code))
                return GoHomeResponse(success="ERROR")

        # Get the relative movement to the cornstalk
        delta = tfBuffer.lookup_transform('corn_cam', 'gripper', rospy.Time(), rospy.Duration(3.0)).transform.translation
        del_x, del_y, del_z = -delta.x * 1000, -delta.y * 1000, -delta.z * 1000 
        del_x += (del_x - 0.144) * 0.026 + 0.05

        del_y -= 10

        # Move to pre-grasp 1/2
        x_mov, y_mov, z_mov = del_x-85, del_y, del_z
        
        # stored for unhook "pre-grasp"
        
        # MODIFIED
        self.x_mov_unhook, self.y_mov_unhook, self.z_mov_unhook = -x_mov, -y_mov, -z_mov
        # code = self.arm.set_position_aa(axis_angle_pose=[x_mov, y_mov, z_mov, 0, 0, 0], speed=50, relative=True, wait=True)
        code = self.arm.set_position_aa(axis_angle_pose=[0, 0, z_mov, 0, 0, 0], speed=50, relative=True, wait=True)
        code = self.arm.set_position_aa(axis_angle_pose=[x_mov, 0, 0, 0, 0, 0], speed=50, relative=True, wait=True)

        # sys.exit()
        if code != 0:
            rospy.logerr("set_arm_position_aa returned error {}".format(code))
            return HookCornResponse(success="ERROR")
        
        # Move to pre-grasp 2/2
        code = self.arm.set_position_aa(axis_angle_pose=[0, y_mov, 0, 0, 0, 0], speed=30, relative=True, wait=True)

        if code != 0:
            rospy.logerr("set_arm_position_aa returned error {}".format(code))
            return HookCornResponse(success="ERROR")
        
        # Move to grasp
        code = self.arm.set_position_aa(axis_angle_pose=[85, 0, 0, 0, 0, 0], speed=30, relative=True, wait=True)

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
        code = self.arm.set_position_aa(axis_angle_pose=[del_x, del_y, 0, 0, 0, self.absolute_angle], speed=50, relative=True, wait=True, is_radian=False)
        
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

        code = self.arm.set_position_aa(axis_angle_pose=[self.unhook_x, self.unhook_y, 0, 0, 0, -self.absolute_angle], speed=50, relative=True, wait=True, is_radian=False)
        
        if code != 0:
            rospy.logerr("set_arm_position_aa returned error {}".format(code))
            return UnhookCornResponse(success="ERROR")

        # Move to ungrasp
        code = self.arm.set_position_aa(axis_angle_pose=[-85, 0, 0, 0, 0, 0], speed=50, relative=True, wait=True)

        if code != 0:
            rospy.logerr("set_arm_position_aa returned error {}".format(code))
            return UnhookCornResponse(success="ERROR")
        
        # Move to pre-grasp
        code = self.arm.set_position_aa(axis_angle_pose=[0, self.y_mov_unhook, 0, 0, 0, 0], speed=50, relative=True, wait=True)

        if code != 0:
            rospy.logerr("set_arm_position_aa returned error {}".format(code))
            return UnhookCornResponse(success="ERROR")
        
        # MODIFIED
        # code = self.arm.set_position_aa(axis_angle_pose=[self.x_mov_unhook, self.y_mov_unhook, self.z_mov_unhook, 0, 0, 0], speed=50, relative=True, wait=True)
        code = self.arm.set_position_aa(axis_angle_pose=[self.x_mov_unhook, 0, 0, 0, 0, 0], speed=50, relative=True, wait=True)
        code = self.arm.set_position_aa(axis_angle_pose=[0, 0, self.z_mov_unhook, 0, 0, 0], speed=50, relative=True, wait=True)

        if code != 0:
            rospy.logerr("set_arm_position_aa returned error {}".format(code))
            return UnhookCornResponse(success="ERROR")
        
        # Joint angles corresponding to end-effector facing the left side of the amiga base
        code = self.arm.set_servo_angle(angle=[0, -90, 0, -90, 90, 0], is_radian=False, wait=True)

        if code != 0:
            rospy.logerr("set_servo_angle returned error {}".format(code))
            return UnhookCornResponse(success="ERROR")

        self.state = "HOME"
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

        if self.state != "CORN_OFFSET":
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
        
        if code != 0:
            rospy.logerr("set_arm_position_aa returned error {}".format(code))
            return ArcCornResponse(success="ERROR")

        self.absolute_angle += req.relative_angle

        self.state = "CORN_OFFSET"
        return ArcCornResponse(absolute_angle=self.absolute_angle, success="DONE")
    

if __name__ == '__main__':
    rospy.init_node('nimo_manipulation')
    detect_node = xArm_Motion('192.168.1.196')
    rospy.spin()
