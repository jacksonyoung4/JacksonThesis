# General libraries
import sys
import rospy
import actionlib
import math

# For file saving
import os
import re

# Import MoveIt libs
import moveit_commander
from moveit_commander import RobotCommander, MoveGroupCommander, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
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

def save_pose(current: PoseStamped, pose_number: int):
    
    if not os.path.exists("baxter_poses"):
        os.makedirs("baxter_poses")
    
    save_path = os.path.join("baxter_poses", f"{pose_number}.txt")

    with open(save_path, "w") as f:

        f.write(str(current.pose.position.x)+"\n")
        f.write(str(current.pose.position.y)+"\n")
        f.write(str(current.pose.position.z)+"\n")
        f.write(str(current.pose.orientation.x)+"\n")
        f.write(str(current.pose.orientation.y)+"\n")
        f.write(str(current.pose.orientation.z)+"\n")
        f.write(str(current.pose.orientation.w)+"\n")

if __name__ == "__main__":
    
    untucked = [-0.077, -0.998, -1.192, 1.940, 0.672, 1.032, -0.499]

    untucked_high = [0.064, -1.116, -1.365, 1.777, 0.511, 1.266, -0.361]

    joints1 = [-0.447, -0.678, -1.054, 1.205, 0.358, 1.063, -0.549]

    joints2 = [-0.446, -0.722, -0.853, 1.014, 0.752, 1.032, -0.492]

    joints3 = [-0.095, -0.695, -0.937, 1.063, 0.667, 1.027, -1.608]

    joints4 = [-0.604, -1.182, -0.197, 1.716, 0.831, -0.029, 0.479]

    joints5 = [-0.112, -1.106, -1.259, 1.698, 0.516, 0.841, -1.286]



    joints_list = [untucked, untucked_high, joints1, joints2, joints3, joints4, joints5, untucked_high, untucked]

    for i, joints in enumerate(joints_list, 1):

        left_arm_group.set_start_state_to_current_state()
        left_arm_group.set_joint_value_target(joints)
        ok = left_arm_group.go(wait=True)
        left_arm_group.stop()
        left_arm_group.clear_pose_targets()
        
        if not ok:
            print(f"Failed to move to joints {i}")
            continue
        
        rospy.sleep(2.0)

        current = left_arm_group.get_current_pose()
        save_pose(current, i)

        rospy.sleep(2.0)
    
    # left_arm_group.set_start_state_to_current_state()
    # left_arm_group.set_joint_value_target(untucked)
    # ok = left_arm_group.go(wait=True)
    # left_arm_group.stop()
    # left_arm_group.clear_pose_targets()

    moveit_commander.roscpp_shutdown()
    sys.exit(0)

   

#def move_to_pose(move_group: MoveGroupCommander, pose: Pose):
   # """
   # Moves the specified MoveGroup to the given cartesian pose.
   # :param move_group: MoveGroupCommander for the arm
   # :param pose: Pose to move to
   # """
   # move_group.set_pose_target(pose)
   # move_group.go(wait=True)
   # #plan = move_group.go(wait=True)
   # move_group.stop()
   # move_group.clear_pose_targets()

# if __name__ == "__main__":
#     current = left_arm_group.get_current_pose()
#     rospy.sleep(2.0)
#     print(current)

#     save_path = get_next_filename("baxter_poses")

#     with open(save_path, "w") as f:

#         f.write(str(current.pose.position.x)+"\n")
#         f.write(str(current.pose.position.y)+"\n")
#         f.write(str(current.pose.position.z)+"\n")
#         f.write(str(current.pose.orientation.x)+"\n")
#         f.write(str(current.pose.orientation.y)+"\n")
#         f.write(str(current.pose.orientation.z)+"\n")
#         f.write(str(current.pose.orientation.w)+"\n")

#     print(f"Pose saved to {save_path}")
#     #current.pose.position.z += 0.1 
#     #left_arm_group.set_pose_target(current)
#     #left_arm_group.go(wait=True)
#     #left_arm_group.stop()
#     #left_arm_group.clear_pose_targets()
#     moveit_commander.roscpp_shutdown()
#     sys.exit(0)



