import numpy as np
import os
from pr3_utils import *
from helpers import *
from scipy.linalg import expm
from numpy.linalg import inv


def ekf_slam(
    # EKF data
    t, features, linear_velocity, angular_velocity, imu_T_cam, K, b,
    # EKF parameters
    linear_vel_variance=None, angular_vel_variance=None,
    v_weight=1000,
    verbose=0, is_visual_inertial=False,
):
    Ks = calc_Ks(K, b)
    cam_T_imu = inv(imu_T_cam)

    nt = t.shape[0]
    n_lmk_obs = features.shape[1]

    # mean and covariance of the IMU pose
    mu = np.eye(4)
    Sigma = np.eye(6)

    if linear_vel_variance is None or angular_vel_variance is None:
        W = np.diag([5e-2] * 3 + [4e-2] * 3)
    else:
        W = np.diag(np.concatenate((linear_vel_variance, angular_vel_variance)))

    pose_trajectory = np.zeros((nt, 4, 4))
    pose_trajectory[0] = inv(mu)

    # mean and covariance of the landmark position
    mu_l = np.full((n_lmk_obs, 4), np.nan)
    Sigma_l = np.zeros((n_lmk_obs, 3, 3))
    for i in range(n_lmk_obs):
        Sigma_l[i] = np.eye(3) * 100

    V = np.eye(4) * v_weight
    lmk_trajectory = -np.ones((nt, n_lmk_obs, 3))

    P = np.eye(3, 4)

    for i in range(1, nt):
        tau = t[i] - t[i - 1]
        z_ti = features[i]

        # (a) IMU Localization via EKF Prediction
        v = linear_velocity[i]
        w = angular_velocity[i]
        u = np.concatenate((v, w))
        u_hat = hat6(u)
        u_wedge = wedge(u)

        # predicted pose and covariance
        mu = expm(-tau * u_hat) @ mu  # should be -tau
        Sigma = expm(-tau * u_wedge) @ Sigma @ expm(-tau * u_wedge).T + W

        # mu is imu_T_world
        word_T_imu = inv(mu)
        world_T_cam = word_T_imu @ imu_T_cam
        cam_T_world = cam_T_imu @ mu

        dz_list = []
        H_list = []

        # (b) Landmark Mapping via EKF Update
        for j in range(n_lmk_obs):
            # observed landmark position
            zj = z_ti[j]

            # pass the landmark that is not observable
            if np.all(zj == -1):
                continue

            # if the landmark is observed for the first time
            # initialize it with the stereo camera model
            # and pass this loop
            if np.isnan(mu_l[j]).any():
                # pixel to world coordinate
                d = (zj[0] - zj[2])
                z0 = (K[0, 0] * b) / d
                lmk_word_coord = world_T_cam @ np.r_[z0 * inv(K) @ np.r_[zj[:2], 1], 1]
                mu_l[j] = lmk_word_coord
                lmk_trajectory[i, j] = lmk_word_coord[:3]
                continue

            # else calculate predicted pixel observation at t+1 using the landmark position
            lmk_pixel_coord = cam_T_world @ mu_l[j]
            z_tilde = Ks @ pi(lmk_pixel_coord)
            # calculate the Jacobian of the observation model
            H = Ks @ dpi_dq(lmk_pixel_coord) @ cam_T_world @ P.T

            # calculate the Kalman gain for the landmark estimate
            Kg = Sigma_l[j] @ H.T @ inv(H @ Sigma_l[j] @ H.T + V)

            # update the mean and covariance of the landmark
            mu_l[j] = mu_l[j] + P.T @ Kg @ (zj - z_tilde)
            Sigma_l[j] = (np.eye(3) - Kg @ H) @ Sigma_l[j]

            lmk_trajectory[i, j] = mu_l[j][:3]

            # (c) Visual-Inertial SLAM
            if is_visual_inertial:
                mu_m = inv(mu) @ mu_l[j]
                H = -Ks @ dpi_dq(cam_T_imu @ mu_m) @ cam_T_imu @ cdot(mu_m)
                H_list.append(H)
                dz_list.append(zj - z_tilde)

        if verbose > 0 and i % 100 == 0:
            print(i)

        if is_visual_inertial:
            n_observable = len(H_list)
            # if n_observable < 80:
            if n_observable == 0:
                pose_trajectory[i] = word_T_imu
                continue

            if verbose > 1:
                print(f"update visual inertial at i={i}, n_observable={n_observable}")

            H = np.vstack(H_list)
            dz = np.concatenate(dz_list)

            # calculate the Kalman gain for the pose estimate
            Kg = Sigma @ H.T @ inv(H @ Sigma @ H.T + np.kron(np.eye(n_observable), V))
            # perform the inertial EKF update
            mu = expm(hat6(Kg @ dz)) @ mu
            Sigma = (np.eye(6) - Kg @ H) @ Sigma

        pose_trajectory[i] = inv(mu)

    return pose_trajectory, lmk_trajectory, mu, Sigma, mu_l, Sigma_l


if __name__ == '__main__':
    # Load the measurements
    filename = 'data/03.npz'
    t, features, linear_velocity, angular_velocity, K, b, imu_T_cam = load_data(filename)
    t = t.ravel()
    features = features.transpose((2, 1, 0))
    angular_velocity = angular_velocity.T
    linear_velocity = linear_velocity.T
    cam_T_imu = inv(imu_T_cam)

    obs_times = t.shape[0] - np.sum(np.all(features == [-1, -1, -1, -1], axis=2), axis=0)
    idx = obs_times.argsort()[::-1]
    print("max obs times", obs_times[idx[0]], "min obs times", obs_times[idx[2000]])
    features = features[:, idx[:1000], :]
    print(t.shape, features.shape, linear_velocity.shape, angular_velocity.shape)


    lin_vel_var = np.array([1e-6] * 3)
    ang_vel_var = np.array([1e-4] * 3)
    v_weight = 1000
    pose_traj, lmk_traj, mu, Sigma, mu_l, Sigma_l = ekf_slam(
        t, features, linear_velocity, angular_velocity, imu_T_cam, K, b, 
        linear_vel_variance=lin_vel_var, angular_vel_variance=ang_vel_var,
        v_weight=v_weight,
        verbose=1
    )

    pose_traj2, lmk_traj2, mu2, Sigma2, mul_l2, Sigma_l2 = ekf_slam(
        t, features, linear_velocity, angular_velocity, imu_T_cam, K, b, 
        linear_vel_variance=lin_vel_var, angular_vel_variance=ang_vel_var,
        v_weight=v_weight,
        verbose=1, is_visual_inertial=True
    )

    # This function is to visualize the robot pose over time
    label = f"{os.path.basename(filename).split('.')[0]}"
    fig, ax = visualize_trajectory_2d(pose_traj.transpose(1, 2, 0), label, show_ori=True)
    fig2, ax = visualize_trajectory_2d(pose_traj2.transpose(1, 2, 0), label, show_ori=True)

    fig.savefig(f'figure/{label}.png', dpi=300)
    fig2.savefig(f'figure/{label}_visual_inertial.png', dpi=300)
