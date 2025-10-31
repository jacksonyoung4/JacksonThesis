from __future__ import annotations

# Math libraries
import numpy as np
import math
from pathlib import Path
from scipy.spatial.transform import Rotation as R

# General libraries
from geometry_msgs.msg import Pose
import sys
import rospy
import actionlib

# MoveIt libraries
import moveit_commander
from moveit_commander import RobotCommander, MoveGroupCommander, PlanningSceneInterface

# Robitiq libraries
from robotiq_2f_gripper_msgs.msg import CommandRobotiqGripperFeedback, CommandRobotiqGripperResult, CommandRobotiqGripperAction, CommandRobotiqGripperGoal
from robotiq_2f_gripper_control.robotiq_2f_gripper_driver import Robotiq2FingerGripperDriver as Robotiq

# Initialize the MoveIt! commander and rospy node
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('baxter_move_test', anonymous=True)

# Instantiate RobotCommander (interface to the robot)
robot = RobotCommander()

# Instantiate PlanningSceneInterface (interface to the world)
scene = PlanningSceneInterface()

# Group for the left arm
left_arm_group = MoveGroupCommander("left_arm")

# Set the reference frame
left_arm_group.set_pose_reference_frame("base")
left_arm_group.set_planner_id("RRTConnectkConfigDefault")

# Set tolerance and planning time
moveit_tolerance = 0.01
left_arm_group.set_goal_tolerance(moveit_tolerance)
left_arm_group.set_planning_time(10)

# Gripper setup
robotiq_client = actionlib.SimpleActionClient("command_robotiq_action", CommandRobotiqGripperAction)   
robotiq_client.wait_for_server()  
print("Connected to Robotiq gripper")

# Robotiq defined positions for gripper
robotiq_closed = CommandRobotiqGripperGoal()
robotiq_closed.emergency_release = False
robotiq_closed.stop = False
robotiq_closed.position = 0.00
robotiq_closed.speed = 0.1
robotiq_closed.force = 5.0

robotiq_open = CommandRobotiqGripperGoal()
robotiq_open.emergency_release = False
robotiq_open.stop = False
robotiq_open.position = 0.085
robotiq_open.speed = 0.1
robotiq_open.force = 5.0

# Object dimensions dictionary
# Object names must match names of pose files from FoundationPose
dimensions = {
    "vegemite": [0.075, 0.135, 0.048],  # x, y, z
    "polenta": [0.080, 0.150, 0.050],
    "cereal": [0.140, 0.230, 0.060]
}

# Box drop off location
box_position = [0.565, 0.579, 0.193]

# Standard position joint values
untucked = [-0.077, -0.998, -1.192, 1.940, 0.672, 1.032, -0.499]
untucked_high = [0.064, -1.116, -1.365, 1.777, 0.511, 1.266, -0.361]
above_box = [0.578, -0.883, -1.350, 1.498, 0.671, 1.518, 0.073]

# Offsets between gripper frame and actual jaw grasp point (determined based on gripper2target)
z_offset = 0.055
xy_offset = 0.012

# Move left arm to pose
def move_to_pose(position, orientation):
   pose = create_pose(position, orientation)
   left_arm_group.set_pose_target(pose)
   left_arm_group.go(wait=True)
   left_arm_group.stop()
   left_arm_group.clear_pose_targets()

# Create pose
def create_pose(position, orientation):
    pose = Pose()
    pose.position.x = position[0]
    pose.position.y = position[1]
    pose.position.z = position[2]
    pose.orientation.x = orientation[0]
    pose.orientation.y = orientation[1]
    pose.orientation.z = orientation[2]
    pose.orientation.w = orientation[3]
    return pose

# Move left arm to joint angles
def move_to_joints(joints):
    left_arm_group.set_joint_value_target(joints)
    left_arm_group.go(wait=True)
    left_arm_group.stop()
    left_arm_group.clear_pose_targets()

# Determine which object axis is most aligned with z-axis of base
def determine_upright_axis(R):
    if abs(R[2,0]) > abs(R[2,1]) and abs(R[2,0]) > abs(R[2,2]):
        upright_axis = "x"
    elif abs(R[2,1]) > abs(R[2,0]) and abs(R[2,1]) > abs(R[2,2]):
        upright_axis = "y"
    else:
        upright_axis = "z"
    return upright_axis

# Compute the yaw of the object about the z-axis of the base frame
def compute_yaw(R, upright_axis):
    # If x axis is upright, calculate yaw about y axis
    if upright_axis == "x":
        yaw = np.arctan2(R[1,1], R[0,1])
    # If y axis is upright, calculate yaw about x axis
    elif upright_axis == "y":
        yaw = np.arctan2(R[1,0], R[0,0])
    # If z axis is upright, calculate yaw about y axis
    else:
        yaw = np.arctan2(R[1,1], R[0,1])

    # Keep within -90 and +90
    if yaw > math.pi/2:
        yaw -= math.pi
    elif yaw < -math.pi/2:
        yaw += math.pi
    return yaw

# Calculate the quaternion for object grasping based on yaw
def gripper_quat(yaw):
    x = -np.sin(yaw/2)
    y = np.cos(yaw/2)
    z = 0
    w = 0
    return [x, y, z, w]

# Determine height of object (object dimension along its upright axis)
def object_height(upright_axis, object_dimensions):
    if upright_axis == "x":
        dimension = object_dimensions[0]
    elif upright_axis == "y":
        dimension = object_dimensions[1]
    else:
        dimension = object_dimensions[2]
    return dimension

# Determine x and y offsets based on yaw
def gripper_xy_offset(yaw):
    x_offset = xy_offset * np.cos(yaw)
    y_offset = xy_offset * np.sin(yaw)
    return x_offset, y_offset

# Order poses for object grasping based on object closest to box
def order_grasp_poses(grasp_poses):
    distances = []
    # Determine distance between pose and box
    for pose in grasp_poses:
        position = pose.position
        distance_to_box = math.sqrt((box_position[0]-position.x)**2 + (box_position[1]-position.y)**2 + (box_position[2]-position.z)**2)
        distances.append((pose, distance_to_box))
    # Sort by distance
    distances.sort(key=lambda x: x[1])
    # Return poses only
    sorted_poses = [pose for pose, _ in distances]
    return sorted_poses

if __name__ == "__main__":
    
    # Load transformation matrix from camera to base frame
    cam2base = np.loadtxt('cam2base.txt')

    # Load object poses from FoundationPose into a dictionary
    object_poses = {}
    pose_folder = Path('test_poses/')
    for file in pose_folder.glob("*.txt"):
        object_name = file.stem
        object_poses[object_name] = np.loadtxt(file) # Store pose under name of file

    # Determine gripper poses for grasping objects
    grasp_poses = []
    for object_name, pose in object_poses.items():
        
        # Compute pose of target object in robot base frame
        target2cam = pose
        target2base = cam2base @ target2cam
        R = target2base[0:3, 0:3]

        # Determine gripper orientation to align with object for grasping
        upright_axis = determine_upright_axis(R)
        yaw_angle = compute_yaw(R, upright_axis)
        orientation = gripper_quat(yaw_angle)
        
        # Compute gripper position for grasping
        position = target2base[0:3, 3]
        x_offset, y_offset = gripper_xy_offset(yaw_angle)
        position[0] -= x_offset
        position[1] -= y_offset
        position[2] += z_offset

        # Get object height
        object_dimensions = dimensions[object_name]
        height = object_height(upright_axis, object_dimensions)
        # If larger than distance between jaw grip point and top of gripper (4mm)
        if height/2 > 0.04: 
            position[2] += height/2 - 0.04 # Offset grasp from centre of object
        
        # Create pose and add to poses list
        pose = create_pose(position, orientation)
        grasp_poses.append(pose)

    # Order grasp poses by distance to box drop-off
    if len(grasp_poses) > 1:
        ordered_poses = order_grasp_poses(grasp_poses)
    else:
        ordered_poses = grasp_poses

    # High position for safe trajectory
    move_to_joints(untucked_high)
    rospy.sleep(1)

    print("STARTING PICK AND PLACE")

    # Loop for grapsing each object
    for i, pose in enumerate(ordered_poses):

        print(f"Grasping object {i+1}")

        # Get position and orientation from pose
        pose_pos = ordered_poses[i].position 
        position = [pose_pos.x, pose_pos.y, pose_pos.z]
        pose_ori = ordered_poses[i].orientation
        orientation = [pose_ori.x, pose_ori.y, pose_ori.z, pose_ori.w]

        # Go to position 10cm above object
        approach_position = position.copy()
        approach_position[2] += 0.1
        move_to_pose(approach_position, orientation)
        rospy.sleep(1)

        # Go to object pickup position
        move_to_pose(position, orientation)
        rospy.sleep(1)
        robotiq_client.send_goal(robotiq_closed) # Close gripper
        robotiq_client.wait_for_result()
        rospy.sleep(1)

        # Move back to 10cm above object
        move_to_pose(approach_position, orientation)
        rospy.sleep(1)

        # Move above box drop-off location
        move_to_joints(above_box)
        rospy.sleep(1)
        
        # Open gripper
        robotiq_client.send_goal(robotiq_open)
        robotiq_client.wait_for_result()

    # Move back to home position
    move_to_joints(untucked_high)
    rospy.sleep(1)
    move_to_joints(untucked)
    rospy.sleep(1)

    print("PICK AND PLACE COMPLETE")

    # Shut down ROS node
    moveit_commander.roscpp_shutdown()
    sys.exit(0)
