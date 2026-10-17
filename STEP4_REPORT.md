# Step 4 — GPS vs Visual-Inertial Trajectory Comparison

## Setup
The Iris vehicle was simulated in Gazebo Harmonic using ArduPilot SITL. The camera image stream and IMU stream were consumed directly through Gazebo Transport (no ROS bridge). The camera topic was `/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image`; the IMU topic was `/world/iris_runway/model/iris_with_standoffs/link/imu_link/sensor/imu_sensor/imu`.

## Method
1. Camera images are converted to grayscale and sparse features are detected with OpenCV `goodFeaturesToTrack`.
2. Feature correspondence between consecutive frames is estimated using pyramidal Lucas–Kanade optical flow (`calcOpticalFlowPyrLK`). The median pixel displacement is used to reduce the effect of outlier tracks.
3. The IMU gyroscope z-axis is integrated to estimate relative yaw. Image-flow displacement is scaled using an assumed camera height and approximate focal length inferred from a 60-degree horizontal field of view, then accumulated into a relative x-y trajectory.
4. GPS `GLOBAL_POSITION_INT` latitude/longitude are converted to local east/north displacement relative to the first received GPS sample.
5. `plot_comparison.py` plots both tracks after independently normalizing cumulative path length, allowing a trajectory-shape comparison.

## Results

The plotting script generated `gps_vs_vio.png` from `gps_position.csv`
and `vio_estimate.csv`. In the recorded run, the GPS horizontal
displacement remained very small, while the optical-flow estimate
accumulated noticeable relative displacement and drift.

The plot provides a qualitative comparison of the recorded trajectories.
It does not demonstrate that visual odometry is more accurate than GPS.
The current implementation is a prototype, not a validated VIO system.

The recorded `imu_log.csv` was empty, so complete IMU logging and
quantitative validation remain outstanding. No ground-truth trajectory,
ATE/RMSE measurement, or quantitative accuracy claim is provided.

## Limitations
This is a lightweight demonstrator, not a calibrated full 6-DoF estimator or an OpenVINS result. It assumes constant camera height and does not solve monocular metric scale, full roll/pitch compensation, camera–IMU extrinsic calibration, or bias estimation. The plot normalizes path length and therefore compares trajectory shape, not absolute metric accuracy or an ATE/RMSE score. Results depend on visible texture, camera orientation, flight motion and GPS quality.

## Reproduction
See `README.md` for build/run instructions. The relevant source files are `vio_tracker.cpp`, `log_gps.py`, `plot_comparison.py`, and `CMakeLists.txt`.
