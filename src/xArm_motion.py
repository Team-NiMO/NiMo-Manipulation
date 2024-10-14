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
# RobotState, Constraints, OrientationConstraint
import moveit_commander
import tf_conversions

class xArm_Motion():
    @classmethod
    def __init__(self):
        self.loadConfig()

        if self.verbose: rospy.loginfo('Starting xArm_motion node.')
        
        try:
            rospy.wait_for_message("/xarm/joint_states", JointState, timeout=5)
        except rospy.ROSException:
            rospy.logwarn('Unable to connect to xArm')

        # Setup services
        self.get_xArm_service = rospy.Service('GoHome', GoHome, self.GoHome)
        self.get_xArm_service = rospy.Service('GoStow', GoStow, self.GoStow)
        self.get_xArm_service = rospy.Service('LookatCorn', LookatCorn, self.LookatCorn)
        self.get_xArm_service = rospy.Service('LookatAngle', LookatAngle, self.LookatAngle)
        self.get_xArm_service = rospy.Service('GoEM', GoEM, self.GoEM)
        self.get_xArm_service = rospy.Service('GoCorn', GoCorn, self.GoCorn)
        # self.get_xArm_service = rospy.Service('UngoCorn', UngoCorn, self.UngoCorn)
        self.get_xArm_service = rospy.Service('ArcCorn', ArcCorn, self.ArcCorn)
        self.get_xArm_service = rospy.Service('HookCorn', HookCorn, self.HookCorn)
        self.get_xArm_service = rospy.Service('UnhookCorn', UnhookCorn, self.UnhookCorn)

        # Internal variables
        self.state = "HOME" # TODO: This may not be true on startup
        # Angle of xArm relative to the cornstalk
        self.absolute_angle = 0 

        self.broadcaster = tf2_ros.StaticTransformBroadcaster()

        # Setup for MoveIt commands
        self.scene = moveit_commander.PlanningSceneInterface()
        self.robot = moveit_commander.RobotCommander()
        self.group_name = "xarm6"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

        self.pose_goal = geometry_msgs.msg.Pose()
        # self.pose_goal = self.move_group.get_current_pose().pose
        self.pose_goal.orientation = self.robot.get_link('link_eef').pose().pose.orientation 

        self.setupCollisions()

        tfBuffer = tf2_ros.Buffer(rospy.Duration(3.0))
        tf2_ros.TransformListener(tfBuffer)

        if self.verbose: rospy.loginfo('Waiting for service calls...')        
    
    @classmethod
    def setupCollisions(self):
        '''
        Setup collision boxes and planes for environment and end effector
        '''

        # Clear the scene of previous objects
        self.scene.remove_world_object()
        frame = self.robot.get_planning_frame()

        # # Setup end effector box
        # p = geometry_msgs.msg.PoseStamped()
        # p.header.frame_id = "link_eef"
        # p.pose.position.z = 0.11 # EE DIMENSIONS?
        # self.scene.add_box('end_effector', p, size=(0.075, 0.075, 0.075)) # EE DIMENSIONS

        # ---------- For Tabletop Setup ----------
        # Setup ground plane
        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = frame
        p.pose.position.z = 0.91
        self.scene.add_plane("ground", p)

        # Setup table plane
        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = frame
        p.pose.position.x = -0.38
        p.pose.orientation.y = -np.sqrt(2) / 2.0
        p.pose.orientation.w = np.sqrt(2) / 2.0
        self.scene.add_plane("table", p)

        # ---------- For Amiga Setup ----------
        # # Setup ground plane
        # p = geometry_msgs.msg.PoseStamped()
        # p.header.frame_id = self.robot.get_planning_frame()
        # p.pose.position.z = ...
        # self.scene.add_plane("ground", p)

        # # Setup driver's wheel box
        # p = geometry_msgs.msg.PoseStamped()
        # p.header.frame_id = frame
        # p.pose.position.x = ...
        # p.pose.position.y = ...
        # p.pose.position.z = ...
        # self.scene.add_box('end_effector', p, size=(..., ..., ...)) # WHEEL DIMENSIONS

        # # Setup passenger's wheel box
        # p = geometry_msgs.msg.PoseStamped()
        # p.header.frame_id = frame
        # p.pose.position.x = ...
        # p.pose.position.y = ...
        # p.pose.position.z = ...
        # self.scene.add_box('end_effector', p, size=(..., ..., ...)) # WHEEL DIMENSIONS

        # # Setup parallel frame box (frame holding external mechanisms)
        # p = geometry_msgs.msg.PoseStamped()
        # p.header.frame_id = frame
        # p.pose.position.x = ...
        # p.pose.position.y = ...
        # p.pose.position.z = ...
        # self.scene.add_box('end_effector', p, size=(..., ..., ...)) # FRAME DIMENSIONS

        # # Setup perpendicular frame box (frame holding xArm6)
        # p = geometry_msgs.msg.PoseStamped()
        # p.header.frame_id = frame
        # p.pose.position.x = ...
        # p.pose.position.y = ...
        # p.pose.position.z = ...
        # self.scene.add_box('end_effector', p, size=(..., ..., ...)) # FRAME DIMENSIONS

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
    def GoStow(self, req: GoStowRequest) -> GoStowResponse:
        '''
        Move the xArm to the stow position (for navigation)
        
        Returns:
            GoStowResponse: The response:
                           - success - The success of the operation (DONE / ERROR)
        '''

        '''
            #TODO: Figure out what to do to make GoStow work
        '''
        if self.state != "HOME" and self.state != "STOW":
            rospy.logerr("Invalid Command: Cannot move from {} to {}".format(self.state, "STOW"))
            return GoStowResponse(success="ERROR")
        
        if self.verbose: rospy.loginfo('Going to Stow Position')

        joint_goal = self.move_group.get_current_joint_values()
        joint_goal = np.deg2rad([0, -100, 5, 0, 5, -90])

        self.move_group.set_joint_value_target(joint_goal)
        success = self.move_group.go(joint_goal, wait=True)

        self.move_group.stop()
        self.move_group.clear_pose_targets()

        ''' TODO: figure out how we want to do error handling
        if success != True:
            rospy.logerr("set_joint_value_target returned error {}".format(success))
        '''

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

        if self.state == "CORN_HOOK" '''or self.state == "CORN_OFFSET"''':
            rospy.logerr("Invalid Command: Cannot move from {} to {} via GoHome".format(self.state, "HOME"))
            return GoHomeResponse(success="ERROR")
        elif self.state in ["clean", "cal_low", "cal_high"]:
            self.GoEMPlane()
        elif self.state == "LookatCorn":
            self.arm.set_position_aa(axis_angle_pose=[0, -221.5, 0, 6.6, 0, 0], speed=30, relative=True, wait=True)

        if self.verbose: rospy.loginfo('Going to Home Position')

        joint_goal = self.move_group.get_current_joint_values()
        joint_goal = np.deg2rad([0, -90, 0, -90, 90, 0])

        self.move_group.set_joint_value_target(joint_goal)
        success = self.move_group.go(joint_goal, wait=True)

        self.move_group.stop()
        self.move_group.clear_pose_targets()

        ''' TODO: figure out how we want to do error handling
        if success != True:
            rospy.logerr("set_joint_value_target returned error {}".format(success))
        '''

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

        joint_goal = self.move_group.get_current_joint_values()
        joint_goal = np.deg2rad([-90, -112.1, -64.5, 0, 80, -90])

        self.move_group.set_joint_value_target(joint_goal)
        success = self.move_group.go(joint_goal, wait=True)

        self.move_group.stop()
        self.move_group.clear_pose_targets()

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
        
        # Adjust Joint-5 relative to LookatCorn: left (angle > 90°), right (angle < 90°).
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal = np.deg2rad([0, -90, 0, -115, 90-req.joint_angle, 0])

        self.move_group.set_joint_value_target(joint_goal)
        success = self.move_group.go(joint_goal, wait=True)

        self.move_group.stop()
        self.move_group.clear_pose_targets()

        self.state = "LookatAngle"

        return LookatAngleResponse(success="DONE")


    @classmethod
    def GoEMPlane(self):
        '''
        Move the xArm to the external mechanisms plane
        '''
        
        '''
            TODO: Do we need this function anymore?
        '''
        if self.verbose: rospy.loginfo('Going to External Mechanisms Plane')

        joint_goal = self.move_group.get_current_joint_values()
        joint_goal = np.deg2rad([-90, 41.5, -40.3, 0, -88.3, -90])

        self.move_group.set_joint_value_target(joint_goal)
        success = self.move_group.go(joint_goal, wait=True)

        self.move_group.stop()
        self.move_group.clear_pose_targets()

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
        elif self.state not in ["clean", "cal_low", "cal_high"]:
            rospy.logerr("Invalid Command: Cannot move from {} to {}".format(self.state, "EM"))
            return GoEMResponse(success="ERROR")
        
        joint_goal = self.move_group.get_current_joint_values()

        if req.id == "clean":
            if self.verbose: rospy.loginfo("Going to Cleaning Nozzle")

            if self.state == "cal_high":
                # move to the cal_low nozzle before moving to the clean nozzle
                joint_goal = np.deg2rad([-115.3, 89.1, -96, 64.4, -87.6, -102.1])

            # Joint angles corresponding to end-effector at the cleaning nozzle
            joint_goal = np.deg2rad([-125.1, 85, -74.8, 55.2, -96.5, -87.3])

        elif req.id == "cal_low":
            if self.verbose: rospy.loginfo("Going to Low Calibration Nozzle")

            # Joint angles corresponding to end-effector at the low calibration nozzle
            joint_goal = np.deg2rad([-115.3, 89.1, -96, 64.4, -87.6, -102.1])

        elif req.id == "cal_high":
            if self.verbose: rospy.loginfo("Going to High Calibration Nozzle")

            if self.state == "clean":
                # move to the cal_low nozzle before moving to the cal_high nozzle
                joint_goal = np.deg2rad([-115.3, 89.1, -96, 64.4, -87.6, -102.1])
            
            # Joint angles corresponding to end-effector at the high calibration nozzle
            joint_goal = np.deg2rad([-108.7, 110.3, -149.4, 73, -76.8, -129.8])

        self.move_group.set_joint_value_target(joint_goal)
        success = self.move_group.go(joint_goal, wait=True)

        self.move_group.stop()
        self.move_group.clear_pose_targets()
        
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
        # get the current orientation of the end effector
        self.pose_goal.orientation = self.robot.get_link('link_eef').pose().pose.orientation

        self.pose_goal.position.x = req.grasp_point.x
        self.pose_goal.position.y = req.grasp_point.y
        self.pose_goal.position.z = req.grasp_point.z
        self.move_group.set_pose_target(self.pose_goal)

        success = self.move_group.go(self.pose_goal, wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        self.state = "CORN_OFFSET"
        return GoCornResponse(success="DONE")

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

        ''' TODO: Remove xArm API calls
        if req.grasp_point.x < 0.2:
            # include pre grasp pose
            # code = self.arm.set_servo_angle(angle=[-90, -90, 0, -90, 0, 0], is_radian=False, wait=True)

            code = self.arm.set_servo_angle(angle=[0, -90, 0, -90, 90, 0], speed=30, is_radian=False, wait=True)
            if code != 0:
                rospy.logerr("set_servo_angle returned error {}".format(code))
                return HookCornResponse(success="ERROR")
            
            # code = self.arm.set_servo_angle(angle=[-79.4, -81.1, -15.2, -59.5, 12.4, -31.1], speed=30, is_radian=False, wait=True)

            code = self.arm.set_position_aa(axis_angle_pose=[0, -5, 120, 0, 0, 0], speed=30, relative=True, wait=True)

            if code != 0:
                rospy.logerr("set_servo_angle returned error {}".format(code))
                return HookCornResponse(success="ERROR")
        '''
        self.pose_goal  = self.move_group.get_current_pose().pose
        self.pose_goal.orientation = self.robot.get_link('link_eef').pose().pose.orientation 

        # Get the relative movement to the cornstalk
        delta = tfBuffer.lookup_transform('corn_cam', 'gripper', rospy.Time(), rospy.Duration(3.0)).transform.translation
        del_x, del_y, del_z = -delta.x * 1000, -delta.y * 1000, -delta.z * 1000 
        del_x += (del_x - 0.144) * 0.026 + 0.05

        del_y -= 10

        # Move to pre-grasp 1/2
        x_mov, y_mov, z_mov = del_x-85, del_y, del_z
        x_mov += 27
        
        # stored for unhook "pre-grasp"
        self.x_mov_unhook, self.y_mov_unhook, self.z_mov_unhook = -x_mov, -y_mov, -z_mov

        # move to pre-grasp
        self.pose_goal.position.x = x_mov
        self.pose_goal.position.y = y_mov
        self.pose_goal.position.z = z_mov

        self.move_group.set_pose_target(self.pose_goal)

        success = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        '''
        # code = self.arm.set_position_aa(axis_angle_pose=[x_mov, y_mov, z_mov, 0, 0, 0], speed=30, relative=True, wait=True)
        code = self.arm.set_position_aa(axis_angle_pose=[0, 0, z_mov, 0, 0, 0], speed=30, relative=True, wait=True)
        code = self.arm.set_position_aa(axis_angle_pose=[x_mov, 0, 0, 0, 0, 0], speed=30, relative=True, wait=True)
        

        self.move_group.set_pose_target(self.pose_goal)
        success = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        if code != 0:
            rospy.logerr("set_arm_position_aa returned error {}".format(code))
            return HookCornResponse(success="ERROR")
        
        # Move to pre-grasp 2/2
        code = self.arm.set_position_aa(axis_angle_pose=[0, y_mov, 0, 0, 0, 0], speed=30, relative=True, wait=True)

        if code != 0:
            rospy.logerr("set_arm_position_aa returned error {}".format(code))
            return HookCornResponse(success="ERROR")
        '''

        # Move to grasp
        code = self.arm.set_position_aa(axis_angle_pose=[85, 0, 0, 0, 0, 0], speed=30, relative=True, wait=True)

        if code != 0:
            rospy.logerr("set_arm_position_aa returned error {}".format(code))
            return HookCornResponse(success="ERROR")
        
        ''' TODO: See how to modify this in coordination with ArcCorn with moveit
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
        '''

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

        # Move to ungrasp
        code = self.arm.set_position_aa(axis_angle_pose=[-85, 0, 0, 0, 0, 0], speed=30, relative=True, wait=True)

        if code != 0:
            rospy.logerr("set_arm_position_aa returned error {}".format(code))
            return UnhookCornResponse(success="ERROR")
        
        # Move to pre-grasp
        code = self.arm.set_position_aa(axis_angle_pose=[0, self.y_mov_unhook, 0, 0, 0, 0], speed=30, relative=True, wait=True)

        if code != 0:
            rospy.logerr("set_arm_position_aa returned error {}".format(code))
            return UnhookCornResponse(success="ERROR")
        
        # MODIFIED
        # code = self.arm.set_position_aa(axis_angle_pose=[self.x_mov_unhook, self.y_mov_unhook, self.z_mov_unhook, 0, 0, 0], speed=30, relative=True, wait=True)
        code = self.arm.set_position_aa(axis_angle_pose=[self.x_mov_unhook, 0, 0, 0, 0, 0], speed=30, relative=True, wait=True)
        code = self.arm.set_position_aa(axis_angle_pose=[0, 0, self.z_mov_unhook, 0, 0, 0], speed=30, relative=True, wait=True)

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

        # Create a cartesian path to arc
        waypoints = []
        resolution = 1.0 # Arc resolution in degrees

        res = -resolution if req.relative_angle < 0 else resolution

        for ang in np.arange(res, req.relative_angle, res):
            c_x = x_curr - (0.1 + radius) * np.sin(np.radians(self.absolute_angle))
            c_y = y_curr - (0.1 + radius) * np.cos(np.radians(self.absolute_angle))

            # Determine the offset to move in x and y
            x_pos = c_x + (0.1 + radius) * np.sin(np.radians(ang + self.absolute_angle))
            y_pos = c_y + (0.1 + radius) * np.cos(np.radians(ang + self.absolute_angle))
            del_x, del_y = (x_pos-x_curr), (y_pos-y_curr)

            # Convert to pose
            pose_goal = self.move_group.get_current_pose().pose
            
            # Update orientation
            quaternion = (
                pose_goal.orientation.x,
                pose_goal.orientation.y,
                pose_goal.orientation.z,
                pose_goal.orientation.w 
            )
            euler = list(tf_conversions.transformations.euler_from_quaternion(quaternion))
            euler[2] += np.deg2rad(-ang)
            quaternion = tf_conversions.transformations.quaternion_from_euler(euler[0], euler[1], euler[2])
            
            pose_goal.position.x += del_x
            pose_goal.position.y += del_y
            pose_goal.orientation.x = quaternion[0]
            pose_goal.orientation.y = quaternion[1]
            pose_goal.orientation.z = quaternion[2]
            pose_goal.orientation.w = quaternion[3]

            waypoints.append(pose_goal)

        plan, _ = self.move_group.compute_cartesian_path(waypoints, 0.01, jump_threshold=5)
        self.move_group.execute(plan, wait=True)

        pose = self.move_group.get_current_pose().pose
        quaternion = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w 
        )
        euler = list(tf_conversions.transformations.euler_from_quaternion(quaternion))
        self.absolute_angle = int(np.round(-np.rad2deg(euler[2])))

        self.state = "CORN_OFFSET"
        return ArcCornResponse(absolute_angle=self.absolute_angle, success="DONE")

if __name__ == '__main__':
    rospy.init_node('nimo_manipulation')
    detect_node = xArm_Motion()
    rospy.spin()
