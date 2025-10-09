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

def move_to_pose(move_group: MoveGroupCommander, pose: Pose):
   # """
   # Moves the specified MoveGroup to the given cartesian pose.
   # :param move_group: MoveGroupCommander for the arm
   # :param pose: Pose to move to
   # """
   move_group.set_pose_target(pose)
   move_group.go(wait=True)
   #plan = move_group.go(wait=True)
   move_group.stop()
   move_group.clear_pose_targets()

def rotate_quat_z(angle):
    rad = np.deg2rad(angle)
    x = -np.sin(rad/2)
    y = np.cos(rad/2)
    z = 0
    w = 0
    return [x, y, z, w]

if __name__ == "__main__":
    cam2base = np.loadtxt('cam2base.txt')
    gripper2target = np.loadtxt('gripper2target.txt')
    target2cam = np.loadtxt('target2cam.txt')

    gripper2base = cam2base @ target2cam @ gripper2target

    position = gripper2base[0:3, 3]
    #rot = R.from_matrix(gripper2base[0:3, 0:3])
    #orientation = rot.as_quat()  # x, y, z, w
    #orientation = [0,0,0,1]
    orientation = rotate_quat_z(-90)
    pose = create_pose(position, orientation)
    move_to_pose(left_arm_group, pose)

    moveit_commander.roscpp_shutdown()
    sys.exit(0)
