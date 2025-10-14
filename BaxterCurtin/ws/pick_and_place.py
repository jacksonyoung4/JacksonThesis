# adding 3.9+ style list[type] annotations
from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PoseStamped, Pose

# General libraries
import sys
import rospy
import actionlib

# Import MoveIt libs
import moveit_commander
from moveit_commander import RobotCommander, MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject

# Import Robitiq libs
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

moveit_tolerance = 0.01
left_arm_group.set_goal_tolerance(moveit_tolerance)

# Gripper setup
robotiq_client = actionlib.SimpleActionClient("command_robotiq_action", CommandRobotiqGripperAction)   
robotiq_client.wait_for_server()  
print("Connected to Robotiq gripper")

# Robotiq defined positions
robotiq_closed = CommandRobotiqGripperGoal()
robotiq_closed.emergency_release = False
robotiq_closed.stop = False
robotiq_closed.position = 0.00
robotiq_closed.speed = 0.1
robotiq_closed.force = 1.0

robotiq_open = CommandRobotiqGripperGoal()
robotiq_open.emergency_release = False
robotiq_open.stop = False
robotiq_open.position = 0.085
robotiq_open.speed = 0.1
robotiq_open.force = 5.0

def move_to_pose(position, orientation):
   pose = create_pose(position, orientation)
   left_arm_group.set_pose_target(pose)
   left_arm_group.go(wait=True)
   left_arm_group.stop()
   left_arm_group.clear_pose_targets()

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

def rotate_quat_z(angle):
    x = -np.sin(angle/2)
    y = np.cos(angle/2)
    z = 0
    w = 0
    return [x, y, z, w]

def move_to_joints(joints):
    left_arm_group.set_joint_value_target(joints)
    left_arm_group.go(wait=True)
    left_arm_group.stop()
    left_arm_group.clear_pose_targets()

if __name__ == "__main__":
    
    # Load transformation matrices
    cam2base = np.loadtxt('cam2base.txt')
    gripper2target = np.loadtxt('gripper2target.txt')
    target2cam = np.loadtxt('target2cam.txt')

    # Compute yaw of object about z axis in base frame - gripper aligns for pickup
    target2base = cam2base @ target2cam
    yaw_angle = np.arctan2(target2base[1,0], target2base[0,0])
    orientation = rotate_quat_z(yaw_angle)

    # Compute gripper position for pickup
    gripper2base = target2base @ gripper2target
    position = gripper2base[0:3, 3]

    # Starting positon joint values
    untucked = [-0.077, -0.998, -1.192, 1.940, 0.672, 1.032, -0.499]
    untucked_high = [0.064, -1.116, -1.365, 1.777, 0.511, 1.266, -0.361]
    
    # Safe start movements
    move_to_joints(untucked)
    rospy.sleep(1)
    move_to_joints(untucked_high)
    rospy.sleep(1)
    # Open gripper
    robotiq_client.send_goal(robotiq_open)
    robotiq_client.wait_for_result()

    # Go to position 10cm above object
    approach_position = position.copy()
    approach_position[2] += 0.1
    move_to_pose(approach_position, orientation)
    rospy.sleep(1)

    # Go to object pickup position
    move_to_pose(position, orientation)
    rospy.sleep(1)
    robotiq_client.send_goal(robotiq_closed)
    robotiq_client.wait_for_result()
    rospy.sleep(1)

    # Move back to 10cm above object
    move_to_pose(approach_position, orientation)
    rospy.sleep(1)

    # Move back to start
    move_to_joints(untucked_high)
    rospy.sleep(1)
    move_to_joints(untucked)
    rospy.sleep(1)
    # Open gripper
    robotiq_client.send_goal(robotiq_open)
    robotiq_client.wait_for_result() 

    moveit_commander.roscpp_shutdown()
    sys.exit(0)
