# BaxterCurtin

## Directory Structure
- For explanation on the directory structure see:   
https://github.com/nathan-broadbent/BaxterCurtin/blob/main/README.md
- All code relevant to this thesis is contained in ws/jackson_src

## Using Baxter
### Starting Robot
- Turn on at power point (yellow power cord)
- Open terminal
``` bash
cd Jackson/BaxterCurtin/ws
```
- Connect to baxter
``` bash
. baxter.sh
. devel/setup.sh
```
- Enable robot
``` bash
rosrun baxter_tools enable_robot.py -e 
```
- Untuck arms:
``` bash
rosrun baxter_tools tuck_arms.py -u 
```
- Launch action server to allow moveit to actually move robot
``` bash
rosrun baxter_interface joint_trajectory_action_server.py 
``` 
### Starting MoveIt
- Open a new terminal
``` bash
. baxter.sh
. devel/setup.sh
```
- Launch MoveIt
``` bash
roslaunch baxter_moveit_config demo_baxter.launch
```
### Connecting to 2F-85 Gripper
- Open a new terminal
``` bash
. baxter.sh
. devel/setup.sh
```
- Launch gripper
``` bash
roslaunch robotiq_2f_gripper_control robotiq_action_server.launch
```
### Executing Script
- Open terminal
- Connect to baxter
``` bash
. baxter.sh
. devel/setup.sh
```
- Run script with commandline remap
``` bash
python script_name.py /joint_states:=/robot/joint_states
```

### Shutting Down Robot
- Kill all processes (ctrl+c)
- Tuck arms, untuck arms, and disable robot.
- Turn off at power point

## Hand-Eye Calibration
### Data Collection
-  Grip the target object with the gripper or fix it to the gripper.
- Try to align the centre of the object with the grasp point of the jaws.
- Execute get_gripper_calib_poses.py while recording with record_for_FoundationPose.py.
- Poses of the gripper are saved to gripper2base directory.
- Add mesh to recording and use as input for FoundationPose to track object pose for entire calibration.
- Take the object pose from frames where gripper and object are stationary in each of the 20 poses.
- Add the poses to target2cam directory and rename them 1-20 (keeping original order).

### Calibration
- Execute calibrate_hand_eye.py to obtain transformations cam2base and gripper2target.
- Execute hand_eye_calibration_error.py to assess the accuracy of the calibration.

## Pick-and-Place
- Place objects on the table and record them with capture_pose.py.
- Use recording and object mesh as input for FoundationPose.
- Make sure the coordinate frame in object mesh is defined such that:
    * If the x or z-axis of the object is most aligned with the z-axis of the base frame, it is optimal to grasp the object along its y-axis.
    * If the y-axis of the object is most aligned with the z-axis of the base frame, it is optimal to grasp the object along its x-axis.
- Put pose estimates from FoundationPose in test_poses directory with the pose having the same name as the object.
- The following adjustments may need to be made to the pick_and_place.py script:
    * If new objects, add them and their dimensions to the dimensions dictionary.
    * If box is in a new position, adjust box coordinates and joint angles.
- If new calibration:
    * Add the cam2base and gripper2target transformations to the pick_and_place directory.
    * Adjust xy and z offsets in pick_and_place.py based on gripper2target transformation.