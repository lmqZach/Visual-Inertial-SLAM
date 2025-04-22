
# Visual-Inertial SLAM

**Author:** Zach(Muqing) Li  

---

## SLAM Overview
This project implements a Visual-Inertial SLAM system using stereo camera, gyroscope, and accelerometer data to perform simultaneous localization and mapping with an Extended Kalman Filter (EKF). The app provides a user-friendly 3D interface for playback and inspection of the SLAM process.

### Methodology
Technically, the goal is to implement an EKF prediction step based on SE(3) kinematics with IMU measurements and an EKF update step based on the stereo-camera observation model with feature observations to perform localization and mapping.

Methodology breakdown:
1. IMU Localization via EKF Prediction: Implement EKF prediction based on SE(3) kinematics with linear and angular velocity measurements to estimate IMU's poses Tt ∈ SE(3) over time t.
2. Landmark Mapping via EKF Update: Implement EKF with the unknown landmark positions m ∈ R^(3×M) as a state and perform EKF update after every visual observation zt to keep track of mean and covariance of m. 
3. Visual-Inertial SLAM: Combine IMU prediction step from (1), with the landmark update step from (2), and implement IMU update step based on the stereo-camera observation model to complete visual-inertial SLAM algorithm.

---

## File Structure

```
Visual-Inertial-SLAM/
│
├── app.py                        # Streamlit app for SLAM playback and visualization
├── main.py                       # Core script to run SLAM logic
├── ekf_slam_stepper.py           # EKF SLAM step-by-step processor
├── helpers.py                    # Projection and stereo camera matrix utilities
├── pr3_utils.py                  # Provided utility functions for stereo geometry
├── requirements.txt              # Required Python packages
├── README.md                     # Project overview and execution guide
├── Visual-Inertial-SLAM_Report.pdf   # Technical project report
├── LICENSE                       # License file
│
├── data/                         # Input data files (.npz format)
│   ├── 03.npz
│   └── 10.npz
│
├── figure/                       # Output visualizations
│   ├── 03.png
│   └── 03_visual_inertial.png
│   └── saved/
│       ├── 03.png
│       ├── 03_compare.png
│       ├── 03_visual_inertial.png
│       └── 10.png
│
└── .git/                         # Git metadata (hidden folder)
```

---

## How to Run

To run the visualization tool:
```bash
cd Visual-Inertial-SLAM
streamlit run app.py
```

To run the baseline SLAM script:
```bash
python main.py
```

---

## SLAM Visualization App Functionality

This application demonstrates a visual-inertial SLAM engine for real-time 3D pose estimation and environmental mapping. It fuses stereo camera data, gyroscope, and accelerometer measurements through an Extended Kalman Filter (EKF) that operates in SE(3) space, allowing for simultaneous motion tracking and world-frame landmark estimation.

The tool features a full 3D visualization of the evolving robot pose and environment map, with both manual step-by-step and automatic playback modes. This serves as a high-fidelity simulation and diagnostic interface for autonomous navigation research.

### The Streamlit app provides:
- **Manual Playback**: Step through SLAM results frame by frame with custom delay
- **Autoplay Mode**: Animate the full trajectory with adjustable speed
- **Progress Tracking**: Real-time progress bar during visualization
- **Reset Option**: Reset playback to timestamp 0
- **3D Visualization**: Interactive trajectory and landmarks with Plotly

---

<!-- ## SLAM Overview:
It is important for autonomous vehicles or robots to understand their pose and the real environment. With the data collected from stereo cameras and sensors, it is possible to generate mapping during motion. Given the mapping of the environment, vehicles or robots can perform motions without collision in the environment, and as a result, it will be able to plan trajectories more efficiently. A common technique of performing localization and mapping at the same time is called SLAM. In addition, Extended Kalman Filter is a transformed nonlinear type of Bayes filter to estimate and update the landmarks in the world-frame. -->

## Result and Output

### 2D-Mapping 
Figure 1: IMU Localization from EKF Prediction
<img width="555" alt="Screen Shot 2022-05-09 at 21 22 04" src="https://user-images.githubusercontent.com/92130976/167523826-8f6e4c32-06d8-4cc4-a8d6-52aaee8447ab.png">

Figure 2: Estimated Trajectory with EKF Landmark Mapping
<img width="555" alt="Screen Shot 2022-05-09 at 21 21 22" src="https://user-images.githubusercontent.com/92130976/167523759-6bbb5f5d-b6bf-4bb1-a786-879b7f376f96.png">


### 3D Real-time Tracking and Mapping 
Figure 3: Status Bar and Timestamp Adjustment
<img src="https://github.com/user-attachments/assets/4eb0dc6f-df3f-4b36-80ea-567b51cb2f5e" width="555"/>

Figure 4: SLAM Pose and Estimation
<img src="https://github.com/user-attachments/assets/60530f2c-9b8d-4ec4-bf03-f1702501e06a" width="555"/>



## Key Contributions and Significance
This tool bridges advanced robotics estimation with practical diagnostics and research visualization. It provides an original and impactful contribution to the field of robotics and AI-driven localization and mapping.
- EKF prediction with SE(3) motion modeling
- Landmark update using stereo camera geometry
- Real-time 3D animation with Plotly
- Integration of advanced SLAM logic with a user-driven interface

