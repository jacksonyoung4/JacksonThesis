import numpy as np
import cv2
from scipy.spatial.transform import Rotation

# Get pose of gripper in base frame
def get_gripper2base(file_num):
    position = np.zeros(3)
    quaternion = np.zeros(4)
    f = open(f"gripper2base/{file_num}.txt", "r")
    for i in range(3):
        line = f.readline()
        position[i] = float(line.strip())
    for i in range(4):
        line = f.readline()
        quaternion[i] = float(line.strip())
    f.close()
    R = Rotation.from_quat(quaternion).as_matrix() # Calculate rotation matrix from quaternion
    t = np.array(position)
    return R, t

# Get pose of base in gripper frame
def get_base2gripper(file_num):
    R, t = get_gripper2base(file_num)
    R_inv = R.T
    t_inv = -R_inv @ t
    return R_inv, t_inv

# Get pose of target in camera frame
def get_target2cam(file_num):
    htm = np.loadtxt(f"target2cam/{file_num}.txt")
    R = htm[:3, :3]
    t = htm[:3, 3]
    return R, t

# Calculate the average transformation matrix
def average_transformation(matrices):
    R_sum = np.zeros((3, 3))
    t_sum = np.zeros(3)
    n = len(matrices)

    # Sum rotations and translations
    for mat in matrices:
        R_sum += mat[:3, :3]
        t_sum += mat[:3, 3]

    # Calculate average translation
    t_avg = t_sum / n

    # Perform singular value decomposition to extract only rotation from sum (no scaling/shearing)
    U, _, Vt = np.linalg.svd(R_sum)
    R_avg = U @ Vt

    # Ensure proper rotation (determinant +1)
    if np.linalg.det(R_avg) < 0:
        U[:, -1] *= -1
        R_avg = U @ Vt

    # Assemble average transform
    avg_matrix = np.eye(4)
    avg_matrix[:3, :3] = R_avg
    avg_matrix[:3, 3] = t_avg
    return avg_matrix

if __name__ == "__main__":

    print("STARTING CALIBRATION")

    R_base2gripper = []
    t_base2gripper = []

    R_target2cam = []
    t_target2cam = []

    # Loop through all 20 poses and get R and t
    for i in range (1, 21):

        b2g_R, b2g_t = get_base2gripper(i)
        t2c_R, t2c_t = get_target2cam(i)

        R_base2gripper.append(b2g_R)
        t_base2gripper.append(b2g_t)

        R_target2cam.append(t2c_R)
        t_target2cam.append(t2c_t)

    # Calibrate hand-eye to get cam2base
    R_X, t_X = cv2.calibrateHandEye(R_base2gripper, t_base2gripper, R_target2cam, t_target2cam, method=cv2.CALIB_HAND_EYE_TSAI)
    cam2base = np.eye(4)
    cam2base[:3, :3] = R_X
    cam2base[:3, 3:4] = t_X

    # Save cam2base to txt file
    np.savetxt("cam2base.txt", cam2base)

    # Compute gripper2target
    gripper2target_list = []
    for i in range(1,21):

        # Calculate the position of the target in the base frame
        target2cam = np.loadtxt(f"target2cam/{i}.txt")
        target2base = cam2base @ target2cam

        # Get gripper2base in HTM form (the ideal gripper postion)
        R,t = get_gripper2base(i)
        gripper2base = np.eye(4)
        gripper2base[:3, :3] = R
        gripper2base[:3, 3:4] = t.reshape(3,1)
        
        # Calculate gripper2target for this pose
        gripper2target_i = np.linalg.inv(target2base) @ gripper2base
        gripper2target_list.append(gripper2target_i)

    # Calculate the average gripper2target over all poses
    gripper2target = average_transformation(gripper2target_list)

    # Save gripper2target to txt file
    np.savetxt("gripper2target.txt", gripper2target)

    print("TRANSFORMATIONS SAVED, CALIBRATION COMPLETE!")