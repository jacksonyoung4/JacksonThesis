import numpy as np
import cv2
import matplotlib.pyplot as plt
from calibrate_hand_eye import get_gripper2base, get_base2gripper, get_target2cam, average_transformation 

if __name__ == "__main__":

    R_base2gripper = []
    t_base2gripper = []

    R_target2cam = []
    t_target2cam = []

    t_error_list = []
    R_error_list = []

    # Loop through all 20 poses and get R and t
    for i in range (1, 21):

        b2g_R, b2g_t = get_base2gripper(i)
        t2c_R, t2c_t = get_target2cam(i)

        R_base2gripper.append(b2g_R)
        t_base2gripper.append(b2g_t)

        R_target2cam.append(t2c_R)
        t_target2cam.append(t2c_t)

        # If more than three poses, calibrate based on only those poses
        if i >= 3:
            R_X, t_X = cv2.calibrateHandEye(R_base2gripper, t_base2gripper, R_target2cam, t_target2cam, method=cv2.CALIB_HAND_EYE_TSAI)
            cam2base_i = np.eye(4)
            cam2base_i[:3, :3] = R_X
            cam2base_i[:3, 3:4] = t_X

            # Compute gripper2target
            gripper2target_list = []
            for j in range(1,21):
                # Calculate the position of the target in the base frame
                target2cam = np.loadtxt(f"target2cam/{j}.txt")
                target2base = cam2base_i @ target2cam

                # Get gripper2base in HTM form (the ideal gripper postion)
                R,t = get_gripper2base(j)
                gripper2base = np.eye(4)
                gripper2base[:3, :3] = R
                gripper2base[:3, 3:4] = t.reshape(3,1)
                
                # Calculate gripper2target for this pose
                gripper2target_i = np.linalg.inv(target2base) @ gripper2base
                gripper2target_list.append(gripper2target_i)

            # Calculate the average gripper2target over all poses
            gripper2target = average_transformation(gripper2target_list)

            # Calculate the total rotation and translation errors from all calibration poses
            t_error_total = 0
            R_error_total = 0
            for k in range(1,21):

                # Calulate the predicted gripper pose to grasp the object from the target pose in camera frame
                target2cam = np.loadtxt(f"target2cam/{k}.txt")
                gripper2base_estimated = cam2base_i @ target2cam @ gripper2target
                R_estimated, t_estimated = gripper2base_estimated[:3, :3], gripper2base_estimated[:3, 3]

                # Get the actual gripper pose when grasping the object during calibration
                R_actual,t_actual = get_gripper2base(k)

                # Calculate the rotation error
                R_error_matrix = R_estimated.T @ R_actual
                R_error = np.degrees(np.arccos((np.trace(R_error_matrix) - 1) / 2))
                R_error_total += R_error

                # Calculate the translation error
                t_error = t_estimated - t_actual
                t_error_total += np.linalg.norm(t_error)

            # Calculate the average rotation error
            R_error_avg = R_error_total / i
            R_error_list.append(R_error_avg)

            # Calculate the average translation error
            t_error_avg = t_error_total / i
            t_error_list.append(t_error_avg)
        
        if i == 20:
            print("Average errors for calibrating with all 20 poses:")
            print(f"Rotation error: {R_error_avg}")
            print(f"Translation error: {t_error_avg}")

    # Plot the errors
    # Code adapted from: https://matplotlib.org/stable/gallery/subplots_axes_and_figures/two_scales.html
    poses = np.arange(3, 21)

    fig, ax1 = plt.subplots()
    plt.title('Hand-Eye Calibration Errors vs Number of Poses')

    color = 'tab:blue'
    ax1.set_xlabel('Number of Poses Used')
    ax1.set_ylabel('Mean Absolute Translation Error (m)', color=color)
    ax1.plot(poses, t_error_list, marker='o', color=color)
    ax1.tick_params(axis='y', labelcolor=color)

    ax2 = ax1.twinx()

    color = 'tab:red'
    ax2.set_ylabel('Mean Absolute Rotation Error (degrees)', color=color)
    ax2.plot(poses, R_error_list, marker='s', color=color)
    ax2.tick_params(axis='y', labelcolor=color)

    fig.tight_layout()
    plt.show()