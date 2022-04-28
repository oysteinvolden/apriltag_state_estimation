# apriltag_state_estimation
Fusion of camera-tag pose and IMU using the multiplicative extended kalman filter (MEKF).  


## Overview
This repo contains source code for visual-inertial state estimation. More concrete, an application-specific error-state Kalman filter (MEKF) is implemented with ROS interface. It fuses camera-tag pose (e.g., from AprilTags) with IMU measurements and outputs the full state of the vehicle, including position, velocity, attitude, and biases. 

## Future work

- Update readme (installation, dependencies, basic usage, link to rosbags, ROS topics (input/output))
- make the code more configurable


