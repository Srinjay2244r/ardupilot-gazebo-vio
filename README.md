# ArduPilot SITL + Gazebo: Camera/IMU and VIO Prototype

**Author:** Srinjay Sarkar

## Overview
This project explores simulated UAV sensor integration and a lightweight
visual-odometry baseline using ArduPilot SITL, Gazebo Harmonic, Gazebo
Transport and OpenCV on Ubuntu 22.04.

The Iris vehicle's camera and IMU topics are consumed directly through
Gazebo Transport. Camera features are tracked across frames using
pyramidal Lucas-Kanade optical flow. Gyroscope data is used for relative
yaw estimation, and a GPS logging script records position data for
trajectory comparison.

## Repository Contents
- `vio_quickstart/` — C++ estimator source and CMake build configuration
- `log_gps.py` — GPS position logger
- `plot_comparison.py` — GPS and visual-estimate trajectory plotting
- `camera_log.csv` — recorded camera data
- `gps_position.csv` — recorded GPS positions
- `vio_estimate.csv` — estimated relative trajectory
- `gps_vs_vio.png` — trajectory comparison plot
- `STEP4_REPORT.md` — methodology, results and limitations

## Build
Install the required build and OpenCV dependencies:

    sudo apt install -y build-essential cmake pkg-config libopencv-dev python3-matplotlib

Build the estimator:

    cd ~/vio_gazebo/vio_quickstart
    cmake -S . -B build
    cmake --build build -j1

## Run
Start the Gazebo world and ArduPilot SITL in separate terminals using
your existing simulation setup.

Run the visual-odometry prototype:

    cd ~/vio_gazebo
    ./vio_quickstart/build/vio_tracker 3.0

Record GPS data in another terminal:

    cd ~/vio_gazebo
    PYTHONPATH="$HOME/ardupilot/modules/mavlink" python3 log_gps.py

Generate the comparison plot:

    cd ~/vio_gazebo
    python3 plot_comparison.py

## Limitations
This is an experimental visual-odometry baseline, not a calibrated,
production-grade Visual-Inertial Odometry system or an OpenVINS result.
It assumes a constant camera height and does not solve monocular metric
scale, full roll/pitch compensation, camera-IMU extrinsic calibration
or inertial bias estimation. The plotted trajectories are for qualitative
comparison, not a validated absolute-accuracy measurement.

See `STEP4_REPORT.md` for further details.
