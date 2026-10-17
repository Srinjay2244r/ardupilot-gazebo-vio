# Iris camera + IMU quick VIO baseline (Gazebo Harmonic / Ubuntu 22.04)

This package subscribes directly to Gazebo Transport camera and IMU topics. It logs camera metadata and IMU gyro/acceleration and computes a *lightweight visual-odometry baseline* using image feature tracking, with gyro-integrated yaw. It is useful for demonstrating live sensor subscription and producing a relative GPS-vs-visual trajectory plot quickly.

**Important accuracy note:** This is not a calibrated, production-grade 6-DoF VIO system like OpenVINS. Monocular scale, camera extrinsics, roll/pitch compensation, and full inertial state estimation are not solved. The `cameraHeight` input is an assumed constant scale. The comparison script normalizes both tracks by path length so it compares trajectory shape, not absolute metric accuracy. Be clear about this limitation in the report.

## Build

```bash
sudo apt install -y build-essential cmake pkg-config libopencv-dev python3-matplotlib
cd ~/vio_gazebo
# copy CMakeLists.txt and vio_tracker.cpp here from this folder
cmake -S . -B build
cmake --build build -j1
```

The build uses pkg-config modules `gz-transport13` and `gz-msgs10` that are available on the user's Gazebo Harmonic machine.

## Run with Gazebo

With the `iris_runway` world running in another terminal:

```bash
cd ~/vio_gazebo
./build/vio_tracker 3.0
```

The argument (`3.0`) is the assumed camera-to-ground height in meters used only for approximate optical-flow scale. It creates/overwrites `camera_log.csv`, `imu_log.csv`, and `vio_estimate.csv`. Copy old logs before running if you need to preserve them.

## GPS logging

Ensure SITL/MAVProxy sends MAVLink to UDP `127.0.0.1:14550` (standard `sim_vehicle.py --map --console` setup often does). If `pymavlink` is only installed inside the ArduPilot checkout:

```bash
cd ~/vio_gazebo
PYTHONPATH="$HOME/ardupilot/modules/mavlink" python3 log_gps.py
```

Start GPS logger and `vio_tracker` before takeoff, perform a short controlled motion, then stop both with Ctrl+C. Do not run the OpenVINS compilation while Gazebo and SITL are running on a low-memory machine.

## Plot

```bash
cd ~/vio_gazebo
python3 plot_comparison.py
```

Output: `gps_vs_vio.png`.
