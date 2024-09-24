# NiMo-Manipulation Services

`GoHome` - Return the xArm to it's home position
- Outputs:
    - success: The success of the operation (SUCCESS / ERROR)

***

`LookatCorn` - Rotate the end-effector down so the camera can see the base of the cornstalks
- Outputs:
    - success: The success of the operation (SUCCESS / ERROR)

***

`GoCorn` - Move the end-effector to a position offset from the cornstalk
- Inputs:
    - grasp_point: The position (m) of the grasp point to move to
- Outputs:
    - success: The success of the operation (SUCCESS / ERROR)

***

`UnGoCorn` - Move the end-effector from a position offset from the cornstalk to the home position
- Outputs:
    - success: The success of the operation (SUCCESS / ERROR)

***

`ArcCorn` - Move the end-effector around the cornstalk by a specified angle
- Inputs:
    - relative_angle: The angle to move the end-effector relative to its current position
- Outputs:
    - absolute_angle: The absolute angle of the end-effector about the cornstalk
    - success: The success of the operation (SUCCESS / ERROR)

***

`HookCorn` - Hook the cornstalk at a specified angle
- Inputs:
    - grasp_point: The position (m) of the grasp point for the cornstalk
    - insert_angle: The angle to rotate about the cornstalk before insertion
- Outputs:
    - success: The success of the operation (SUCCESS / ERROR)

***

`UnHookCorn` - Move the end-effector from the grasp position to the home position
- Outputs:
    - success: The success of the operation (SUCCESS / ERROR)

***

`GoEM` - Move the end-effector to the specified pump
- Inputs:
    - id: The specified pump to move to (clean, cal_low, cal_high)
- Outputs:
    - success: The success of the operation (SUCCESS / ERROR)

***

*Replace [NOT IMPLEMENTED]*

*Stow [NOT IMPLEMENTED]*

*LookAtAngle???*