# NiMo-Manipulation
A method to control the UFactory xArm6 for Autonomous Nitrate Monitorring in Cornstalks. More information about the entire system can be found in [NiMo-FSM](https://github.com/Team-NiMO/NiMo-FSM).

## Installation
If the xArm is not already setup, follow the Ufactory xArm6 quick start guide if available, or refer to the [user manual](http://download.ufactory.cc/xarm/en/xArm%20User%20Manual.pdf?v=1578910898247).

First, clone the repository into the `src` folder of your ROS workspace.
```
git clone git@github.com:Team_NiMO/NiMo-Manipulation.git
```

Use [this link](https://github.com/xArm-Developer/xArm-Python-SDK) to install the xArm Python SDK.

Then clone the [xarm_ros](https://github.com/xArm-Developer/xarm_ros) repository into the `src` folder of your ROS workspace.

```
git clone git@github.com:xArm-Developer/xarm_ros.git
catkin_make
```

Calibrate the camera according to the instructions in [NiMo-Perception](https://github.com/Team-NiMO/NiMo-Perception_v2/blob/main/docs/calibration.md).

Additionally, in the file `xarm_ros/xarm6_moveit_config/launch/realMove_exec.launch`, add or modify the static transform for the gripper at the end of the file. Note that the euler angles are zyx, not xyz.

```
<node pkg="tf" type="static_transform_publisher" name="gripper" args="x y z rz ry rz /link_eef /gripper 100"/>
```

## Use
To run the node, you must launch the xArm and run `xArm_motion.py`. This launch file has been created to launch both.

```
roslaunch nimo_manipulation nimo_manipulation.launch
```

The service calls are listed below, more detail can be found in [services.md](/docs/services.md):
- `GoStow` - Position the xArm and end-effector for save travel during locomotion
- `GoHome` - Return the xArm to it's home position
- `LookatCorn` - Position the end-effector so the camera can see the base of the cornstalks
- `LookatAngle` - Rotate the end-effector by a specified angle so the camera can see a wider range of stalks
- `GoCorn` - Move the end-effector to a position offset from the cornstalk
- `ArcCorn` - Move the end-effector around the cornstalk by a specified angle
- `HookCorn` - Hook the cornstalk at a specified angle
- `UnHookCorn` - Move the end-effector from the grasp position to the home position
- `GoEM` - Move the end-effector to the specified pump

## Visualization
Currently, the visualization shows the following:
- A model of the xArm (also representing collision boundaries)
- A model of the end-effector (also representing collision boundaries)
- Collision boundaries for the Amiga Mobile Base and ground plane

<img src="https://github.com/Team-NiMO/Nimo-Manipulation/blob/main/docs/collision_boxes.jpg" width="650">

Note: The visualization is inverted because the arm is mounted upside-down.

## Other
### Resetting after errors
If the xArm is away from the home position when an error occurs and stack needs to be reset, the safeset way to reset the arm is to put it in manual mode via the online interface and bring the arm to a position close to the home position. From there, the FSM can be restarted, or the `GoHome` service can be manually called.

### Changing joint angles for external mechanisms
Since the external mechanisms are at a fixed location, the joint angles to reach each of the nozzles are hard-coded. If the external mechanisms are adjusted, the angles can be adjusted in the function `GoEM` in the script [xArm_motion.py](/src/xArm_motion.py). If all of the nozzles are moved, there should be 5 total changes:
- 1x Clean nozzle
- 3x Low calibration (cal_low) nozzle
- 1x High calibration (cal_high) nozzle

### Future Improvements
- Setting camera and gripper transforms via config file

## Common Issues
**`Invalid Command: Cannot move from ... to ... via ...`** 

In order to minimize errors, the transitions between states are limited. This error means that the commanded transition is not allowed.

**Visualization not tracking xArm**

In some cases, when RViz is opened, the model in the visualization does not have the same configuration as the real xArm. This means that there is some sort of communication error between the arm and the computer. The connections to the arm should be checked and the manipulation node should be restarted.

## Acknowledgements
- [Mark (Moonyoung) Lee](https://github.com/markmlee) for his assistance and advice
- Dr. Oliver Kroemer for his assistance and advice
- Dr. George Kantor for his assistance and advice
- The rest of [Team NiMo](https://github.com/Team-NiMO) for their suppport and feedback
