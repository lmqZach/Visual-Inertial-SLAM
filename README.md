# Run
Run the code in the following way:
```
python main.py
```

# Files
`data/` the data directory for the project.

`figure/` the directory to save the results.

`helper.py`: This file contains the helper functions to construct the projection matrix, rotation matrices, and the stereo camera matrix.

`main.py`: This file contains the main function to run the code.

`README.md`: This file contains the instructions to run the code.

`pr3_utils.py`: This is provided utility package.

`requirements.txt`: This is the list of all required packages to install.


# Results
After running the code, the results will be saved in the `figure` directory.

# Visual-Inertial-SLAM
## Overview:
It is important for autonomous vehicles or robots to understand their pose and the real environment. With the data collected from stereo cameras and sensors, it is possible to generate mapping during motion. Given the mapping of the environment, vehicles or robots can perform motions without collision in the environment, and as a result, it will be able to plan trajectories more efficiently. A common technique of performing localization and mapping at the same time is called SLAM. In addition, Extended Kalman Filter is a transformed nonlinear type of Bayes filter to estimate and update the landmarks in the world-frame.

## Objective
Implement an Extended Kalman Filter to track the three dimensional position and orientation of a robot using gyroscope, accelerometer, and camera measurements. Technically, the goal is to implement an EKF prediction step based on SE(3) kinematics with IMU measurements and an EKF up- date step based on the stereo-camera observation model with feature observations to perform localization and mapping.

## Detailed Tasks
1. IMU Localization via EKF Prediction: Implement the EKF prediction step based on the SE(3) kinematics and the linear and angular velocity measuremetns to estimate the pose Tt ∈ SE(3) of the IMU over time t.


2. Landmark Mapping via EKF Update: implement an EKF with the unknown landmark positions m ∈ R^(3×M) as a state and perform EKF update steps after every visual observation zt in order to keep track of the mean and covariance of m. 


3. Visual-Inertial SLAM: combine the IMU prediction step from part (1), with the landmark update step from part (2) and implement an IMU update step based on the stereo-camera observation model to obtain a complete visual-inertial SLAM algorithm.

## Generated Maps
<img width="555" alt="Screen Shot 2022-05-09 at 21 22 04" src="https://user-images.githubusercontent.com/92130976/167523826-8f6e4c32-06d8-4cc4-a8d6-52aaee8447ab.png">

Figure 1: IMU Localization from EKF Prediction

<img width="558" alt="Screen Shot 2022-05-09 at 21 21 22" src="https://user-images.githubusercontent.com/92130976/167523759-6bbb5f5d-b6bf-4bb1-a786-879b7f376f96.png">

Figure 2: Estimated Trajectory with EKF Landmark Mapping
