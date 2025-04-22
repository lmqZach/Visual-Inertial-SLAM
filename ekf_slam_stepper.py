
import numpy as np
from scipy.linalg import expm
from numpy.linalg import inv
from helpers import calc_Ks, pi, dpi_dq, hat6, wedge

class EKFSLAMStepper:
    def __init__(self, t, features, linear_velocity, angular_velocity, imu_T_cam, K, b,
                 linear_vel_variance=None, angular_vel_variance=None, v_weight=1000):
        self.Ks = calc_Ks(K, b)
        self.cam_T_imu = inv(imu_T_cam)
        self.imu_T_cam = imu_T_cam
        self.K = K
        self.b = b

        self.t = t
        self.features = features
        self.linear_velocity = linear_velocity
        self.angular_velocity = angular_velocity
        self.nt = t.shape[0]
        self.n_lmk_obs = features.shape[1]

        self.mu = np.eye(4)
        self.Sigma = np.eye(6)
        self.mu_l = np.full((self.n_lmk_obs, 4), np.nan)
        self.Sigma_l = np.zeros((self.n_lmk_obs, 3, 3))
        for i in range(self.n_lmk_obs):
            self.Sigma_l[i] = np.eye(3) * 100

        self.V = np.eye(4) * v_weight
        self.W = np.diag([5e-2]*3 + [4e-2]*3) if linear_vel_variance is None or angular_vel_variance is None                  else np.diag(np.concatenate((linear_vel_variance, angular_vel_variance)))

        self.pose_trajectory = [inv(self.mu)]
        self.lmk_trajectory = [-np.ones((self.n_lmk_obs, 3))]

        self.P = np.eye(3, 4)
        self.index = 1

    def step(self):
        if self.index >= self.nt:
            return None  # SLAM complete

        tau = self.t[self.index] - self.t[self.index - 1]
        z_ti = self.features[self.index]

        v = self.linear_velocity[self.index]
        w = self.angular_velocity[self.index]
        u = np.concatenate((v, w))
        u_hat = hat6(u)
        u_wedge = wedge(u)

        self.mu = expm(-tau * u_hat) @ self.mu
        self.Sigma = expm(-tau * u_wedge) @ self.Sigma @ expm(-tau * u_wedge).T + self.W

        word_T_imu = inv(self.mu)
        world_T_cam = word_T_imu @ self.imu_T_cam
        cam_T_world = self.cam_T_imu @ self.mu

        current_landmarks = -np.ones((self.n_lmk_obs, 3))

        for j in range(self.n_lmk_obs):
            zj = z_ti[j]
            if np.all(zj == -1):
                continue

            if np.isnan(self.mu_l[j]).any():
                d = (zj[0] - zj[2])
                z0 = (self.K[0, 0] * self.b) / d
                lmk_word_coord = world_T_cam @ np.r_[z0 * inv(self.K) @ np.r_[zj[:2], 1], 1]
                self.mu_l[j] = lmk_word_coord
                current_landmarks[j] = lmk_word_coord[:3]
                continue

            lmk_pixel_coord = cam_T_world @ self.mu_l[j]
            z_tilde = self.Ks @ pi(lmk_pixel_coord)
            H = self.Ks @ dpi_dq(lmk_pixel_coord) @ cam_T_world @ self.P.T
            Kg = self.Sigma_l[j] @ H.T @ inv(H @ self.Sigma_l[j] @ H.T + self.V)
            self.mu_l[j] = self.mu_l[j] + self.P.T @ Kg @ (zj - z_tilde)
            self.Sigma_l[j] = (np.eye(3) - Kg @ H) @ self.Sigma_l[j]
            current_landmarks[j] = self.mu_l[j][:3]

        self.pose_trajectory.append(inv(self.mu))
        self.lmk_trajectory.append(current_landmarks)
        self.index += 1

        return inv(self.mu), current_landmarks
