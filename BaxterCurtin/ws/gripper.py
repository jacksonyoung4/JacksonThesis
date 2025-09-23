# adding 3.9+ style list[type] annotations
from __future__ import annotations

# General libraries
import sys
import rospy
import actionlib

# Import MoveIt libs
import moveit_commander
from moveit_commander import RobotCommander, MoveGroupCommander, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject

# Import Robitiq libs
from robotiq_2f_gripper_msgs.msg import CommandRobotiqGripperFeedback, CommandRobotiqGripperResult, CommandRobotiqGripperAction, CommandRobotiqGripperGoal
from robotiq_2f_gripper_control.robotiq_2f_gripper_driver import Robotiq2FingerGripperDriver as Robotiq

rospy.init_node('jackson_test', anonymous=True)


robotiq_client = actionlib.SimpleActionClient("command_robotiq_action", CommandRobotiqGripperAction)   
robotiq_client.wait_for_server()  
print("Connected to Robotiq gripper")

# Robotiq defined positions
robotiq_closed = CommandRobotiqGripperGoal()
robotiq_closed.emergency_release = False # Should this be True?
robotiq_closed.stop = False
robotiq_closed.position = 0.00
robotiq_closed.speed = 0.1
robotiq_closed.force = 0.1

robotiq_open = CommandRobotiqGripperGoal()
robotiq_open.emergency_release = False # Should this be True?
robotiq_open.stop = False
robotiq_open.position = 0.085
robotiq_open.speed = 0.1
robotiq_open.force = 5.0

robotiq_client.send_goal(robotiq_open)
robotiq_client.wait_for_result()