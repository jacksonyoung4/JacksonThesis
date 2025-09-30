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

def go_to_joints(move_group: MoveGroupCommander, joints):
    left_arm_group.set_start_state_to_current_state()
    left_arm_group.set_joint_value_target(joints)
    ok = left_arm_group.go(wait=True)
    left_arm_group.stop()
    left_arm_group.clear_pose_targets()
        
    if not ok:
        print(f"Failed to move to joints.")

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

    joints1 = [-0.446, -0.722, -0.853, 1.014, 0.752, 1.032, 1.303]

    joints2 = [-0.447, -0.678, -1.054, 1.205, 0.358, 1.063, -0.549]

    joints3 = [-0.095, -0.695, -0.937, 1.063, 0.667, 1.027, -1.608]

    joints4 = [-0.604, -1.182, -0.197, 1.716, 0.831, -0.029, 0.479]

    joints5 = [-0.112, -1.106, -1.259, 1.698, 0.516, 0.841, -1.286]

    joints6 = [-0.434, -0.940, -0.871, 1.352, 0.738, 1.545, -1.180]
    
    joints7 = [-0.461, -0.670, -0.901, 0.974, 0.717, 1.358, -1.903]

    joints8 = [-0.368, -0.998, -0.975, 1.542, 1.031, 1.224, -0.492]

    joints9 = [-0.425, -0.821, -1.157, 1.217, 0.779, 1.351, 1.881]

    joints10 = [-0.489, -1.124, -0.672, 2.068, 1.984, 0.981, 0.680]

    joints11 = [-0.604, -0.703, -0.816, 1.672, 2.385, 0.714, 0.047]

    joints12 = [-0.050, -0.597, -1.076, 1.790, 2.580, 0.803, -1.055]

    joints13 = [-0.005, -0.807, -1.154, 1.646, 1.977, 0.597, 1.301]

    joints14 = [-0.323, -0.833, -1.363, 1.165, 1.088, 1.422, -0.277]

    joints15 = [0.163, -0.386, -1.593, 0.988, 0.762, 1.573, -1.552]

    joints16 = [-0.342, -0.307, -1.473, 0.721, 0.831, 1.454, 0.576]

    joints17 = [-0.014, -0.596, -1.289, 1.383, 1.192, 0.943, -2.153]

    joints18 = [-0.015, -0.815, -1.542, 1.634, 0.820, 1.394, -0.718]

    joints19 = [-0.163, -0.690, -1.159, 1.028, 1.148, 1.589, -1.512]

    joints20 = [-0.245, -0.658, -1.289, 0.928, 0.938, 1.762, 1.259]

    go_to_joints(left_arm_group, untucked)
    rospy.sleep(1.0)
    go_to_joints(left_arm_group, untucked_high)
    rospy.sleep(1.0)

    joints_list = [joints1, joints2, joints3, joints4, joints5, joints6, joints7, joints8, joints9, joints10, joints11, joints12, joints13, joints14, joints15, joints16, joints17, joints18, joints19, joints20]

    for i, joints in enumerate(joints_list, 1):

        go_to_joints(left_arm_group, joints)

        if i == 1:
            print("Start recording...")
            rospy.sleep(3.5)

        
        rospy.sleep(1.5)

        current = left_arm_group.get_current_pose()
        save_pose(current, i)

        rospy.sleep(0.5)
    
    go_to_joints(left_arm_group, untucked_high)
    rospy.sleep(1.0)
    go_to_joints(left_arm_group, untucked)
    rospy.sleep(1.0)
    
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



