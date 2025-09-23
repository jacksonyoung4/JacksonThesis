# General libraries
import sys
import rospy
import actionlib

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

pose1 = Pose(
    position = Point(0.53, 0.21, 0.17),
    orientation = Quaternion(-0.18, -0.98, -0.02, 0.01)
)

pose2 = Pose(
    position = Point(0.77, 0.01, 0.25),
    orientation = Quaternion(0.65, -0.66, 0.36, 0.13)
)

pose3 = Pose(
    position = Point(0.79, 0.29, 0.30),
    orientation = Quaternion(0.43, 0.89, 0.02, 0.14)
)

pose4 = Pose(
    position = Point(0.90, 0.13, 0.28),
    orientation = Quaternion(0.45, 0.78, -0.14, 0.43)
)

pose5 = Pose(
    position = Point(0.72, 0.50, 0.27),
    orientation = Quaternion(0.52, 0.83, 0.19, 0.08)
)

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
    
    poses =[pose1, pose2, pose3, pose4, pose5]

    for i, pose in enumerate(poses, 1):

        left_arm_group.set_start_state_to_current_state()
        left_arm_group.set_pose_target(pose)
        ok = left_arm_group.go(wait=True)
        left_arm_group.stop()
        left_arm_group.clear_pose_targets()
        
        if not ok:
            print(f"Failed to move to pose {i}")
            continue
        
        rospy.sleep(2.0)

        current = left_arm_group.get_current_pose()
        save_pose(current, i)

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



