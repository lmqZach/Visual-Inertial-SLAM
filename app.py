
import streamlit as st
import numpy as np
import plotly.graph_objs as go
import time
import sys
import os

# Local import path
sys.path.append(os.path.dirname(__file__))
from ekf_slam_stepper import EKFSLAMStepper

# Page setup
st.set_page_config(page_title="SLAM and Viewer", layout="centered")

# ==============================
# Header and App Description
# ==============================
st.markdown("## Simultaneous Localization and Mapping -- Autonumous System SLAM")
st.markdown("**Author:** Zach (Muqing) Li")
st.markdown("**App Overview and Purpose:**")
st.info(
    "This application demonstrates a visual-inertial SLAM engine for real-time 3D pose estimation and mapping. "
    "It fuses stereo camera, gyroscope, and accelerometer data via an Extended Kalman Filter (EKF) operating in SE(3) to simultaneously track motion and update environmental landmarks.\\n\\n"
    )
st.markdown("**Technical Contributions:**")
st.info(
    "The tool supports both manual inspection and continuous trajectory animation for robotics and autonomous systems research.\n\n"
    "- Real-time EKF prediction using inertial measurements over SE(3) motion\n\n"
    "- EKF update using stereo vision for triangulated landmark updates\n\n"
    "- 3D interactive Plotly-based visualization of pose and landmarks\n\n"
)
st.markdown("**Significance and Originality:**")
st.info("This original interface bridges complex SLAM algorithms with practical visualization, enabling deployment and diagnostics in smart mobility and autonomous navigation—core priorities in modern AI and automation.")
st.markdown("---")

# ==============================
# Tabs for Data Input & Playback
# ==============================
tab1, tab2 = st.tabs(["📁 Data Input", "🔍 Playback Visualization"])

with tab1:
    st.markdown("**Description:** Upload your preprocessed SLAM dataset in `.npz` format. The file should include keys like `features`, `time_stamps`, `linear_velocity`, `angular_velocity`, `K`, `b`, and `imu_T_cam`.")

    uploaded_file = st.file_uploader("Upload SLAM Dataset (.npz)", type="npz")

    if "stepper" not in st.session_state:
        st.session_state.stepper = None
        st.session_state.pose_history = []
        st.session_state.landmark_history = []
        st.session_state.timestamps = []
        st.session_state.current_step = 0
        st.session_state.total_steps = 0
        st.session_state.running = False
        st.session_state.autoplay_triggered = False
        st.session_state.loaded_data = {}

    if uploaded_file and st.button("📥 Load Dataset"):
        with np.load(uploaded_file) as data:
            t = data["time_stamps"].squeeze()
            features = data["features"].transpose(2, 1, 0)
            linear_velocity = data["linear_velocity"].T
            angular_velocity = data["angular_velocity"].T
            K = data["K"]
            b = data["b"].item() if isinstance(data["b"], np.ndarray) else data["b"]
            imu_T_cam = data["imu_T_cam"]

            stepper = EKFSLAMStepper(t, features, linear_velocity, angular_velocity, imu_T_cam, K, b)

            st.session_state.stepper = stepper
            st.session_state.pose_history = []
            st.session_state.landmark_history = []
            st.session_state.timestamps = t
            st.session_state.current_step = 0
            st.session_state.total_steps = len(t)
            st.session_state.running = False
            st.session_state.autoplay_triggered = False
            st.session_state.loaded_data = {
                "t": t, "features": features, "linear_velocity": linear_velocity,
                "angular_velocity": angular_velocity, "K": K, "b": b, "imu_T_cam": imu_T_cam
            }

with tab2:
    st.markdown("**Description:** Use this interface to manually step through the SLAM timeline or run the entire playback. You can control playback speed, step size, and reset to start at any time.")

    def step_n_times(n):
        for _ in range(n):
            if st.session_state.current_step >= st.session_state.total_steps:
                break
            result = st.session_state.stepper.step()
            if result:
                pose, landmarks = result
                st.session_state.pose_history.append(pose[:3, 3])
                st.session_state.landmark_history.append(landmarks)
                st.session_state.current_step += 1

    col1, col2 = st.columns(2)
    with col1:
        step_size = st.number_input("Manual step size", min_value=1, max_value=1000, value=1, step=1)
        manual_speed = st.slider("Step delay (ms)", 0, 2000, 200, step=100)
        if st.button("▶ Manual Step"):
            step_n_times(step_size)
            time.sleep(manual_speed / 1000.0)

    with col2:
        autoplay_speed = st.slider("Autoplay speed (ms/frame)", 10, 2000, 500, step=10)
        if st.button("▶ Auto-play to End"):
            st.session_state.running = True
            st.session_state.autoplay_triggered = True

    if st.button("🔁 Reset to Start") and st.session_state.loaded_data:
        st.session_state.stepper = EKFSLAMStepper(
            st.session_state.loaded_data["t"],
            st.session_state.loaded_data["features"],
            st.session_state.loaded_data["linear_velocity"],
            st.session_state.loaded_data["angular_velocity"],
            st.session_state.loaded_data["imu_T_cam"],
            st.session_state.loaded_data["K"],
            st.session_state.loaded_data["b"]
        )
        st.session_state.pose_history = []
        st.session_state.landmark_history = []
        st.session_state.current_step = 0
        st.session_state.running = False
        st.session_state.autoplay_triggered = False

    if st.session_state.total_steps > 0:
        percent = st.session_state.current_step / st.session_state.total_steps
        st.progress(percent, text=f"{int(percent * 100)}% complete")

    if st.session_state.running and st.session_state.autoplay_triggered:
        step_n_times(1)
        time.sleep(autoplay_speed / 1000.0)
        if st.session_state.current_step < st.session_state.total_steps:
            st.rerun()
        else:
            st.session_state.running = False
            st.session_state.autoplay_triggered = False

    if st.session_state.pose_history:
        poses = np.array(st.session_state.pose_history)
        latest_landmarks = st.session_state.landmark_history[-1]
        visible = ~np.all(latest_landmarks == -1, axis=1)

        fig = go.Figure()

        fig.add_trace(go.Scatter3d(
            x=poses[:, 0], y=poses[:, 1], z=poses[:, 2],
            mode='lines+markers', name='Trajectory',
            line=dict(color='blue'), marker=dict(size=2)
        ))

        fig.add_trace(go.Scatter3d(
            x=[poses[-1, 0]], y=[poses[-1, 1]], z=[poses[-1, 2]],
            mode='markers', name='Current Pose',
            marker=dict(size=5, color='red')
        ))

        fig.add_trace(go.Scatter3d(
            x=latest_landmarks[visible, 0],
            y=latest_landmarks[visible, 1],
            z=latest_landmarks[visible, 2],
            mode='markers', name='Landmarks',
            marker=dict(size=2, color='green')
        ))

        fig.update_layout(
            title='SLAM Estimated Pose and Landmarks (3D)',
            scene=dict(xaxis_title='X', yaxis_title='Y', zaxis_title='Z'),
            width=1200, height=900
        )

        st.plotly_chart(fig, use_container_width=True)
