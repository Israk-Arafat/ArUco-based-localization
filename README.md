# JetBot ECE417: ArUco SLAM & Navigation

**Team:** Israk Arafat, Shuvra Smaran Das

## Overview

This project implements a visual localization and navigation system for the NVIDIA JetBot using ArUco markers. Developed for the ECE417 course, the system utilizes a monocular camera to detect markers, estimate the robot's pose in real-time, and build a dynamic map of the environment. It leverages ROS 2 Humble and OpenCV to achieve reliable indoor localization without expensive sensors like LiDAR.

## Key Features

*   **ArUco-based Localization:** Estimates the robot's position and orientation (Pose) relative to a map of known ArUco markers.
*   **Dynamic Mapping:** Features a `map_builder` node that discovers new markers and adds them to the map in real-time, allowing for "SLAM-ish" behavior.
*   **Navigation:** Includes scripts for basic navigation tasks, such as driving towards detected markers (`move2aruco.py`) and patrolling.
*   **Obstacle Avoidance (Planned):** Integration of a CNN-based obstacle detector running on the Jetson Nano.
*   **ROS 2 Integration:** Full usage of the ROS 2 ecosystem, including TF2 for coordinate transforms and standard message types.

![JetBot SLAM Demo](images/jetbot-slam-demo.png)

## Hardware Requirements

*   **NVIDIA JetBot** (Jetson Nano based)
*   **CSI Camera** (Standard JetBot camera)
*   **ArUco Markers** (4x4 Dictionary, 50mm size recommended)

## Software Requirements

*   **OS:** Ubuntu 20.04 (Jetson Nano)
*   **Framework:** ROS 2 Humble
*   **Containerization:** Docker (Recommended for environment consistency)

## Project Structure

```
ws/src/
├── py_pubsub/              # Main project package
│   ├── launch/             # Launch files (SLAM, hardware bringup)
│   ├── py_pubsub/          # Python nodes (Localization, Mapping, Control)
│   └── config/             # Configuration files
├── jetbot_ros/             # JetBot motor control and interface
├── gscam/                  # GStreamer based camera driver
├── ros_aruco_opencv/       # ArUco detection wrapper
└── ...
```

## Installation & Setup

### 1. Connect to the JetBot
SSH into your robot:
```bash
ssh jetbot@<ROBOT_IP>
# Password: jetbot
```

### 2. Start the Docker Container
The project is designed to run inside a ROS 2 Humble Docker container.
```bash
./rundocker.sh
# Or manually:
# docker container exec -it ros-humble bash
```

### 3. Build the Workspace
Inside the container, navigate to the workspace and build:
```bash
cd /home/nihal/jetbot-ece417/ws
colcon build --packages-select py_pubsub jetbot_ros
source install/setup.bash
```

## Usage

### Running Dynamic SLAM
To start the camera, ArUco detector, and the dynamic mapping/localization system:

```bash
ros2 launch py_pubsub dynamic_slam.launch.xml
```
*This will start the `map_builder` and `aruco_localizer` nodes. The first marker detected will become the map origin (0,0,0).*

### Robot Control
To enable motor control, ensure the motor driver is running (if not included in the main launch):
```bash
ros2 run jetbot_ros motors_waveshare
```

To run the "Move to ArUco" behavior:
```bash
ros2 run py_pubsub move2aruco
```

## Nodes Description

*   **`map_builder`**: Listens for marker detections. If a marker is seen that isn't in the map, it calculates its position relative to the robot and adds it to the map.
*   **`aruco_localizer`**: Uses the known map of markers to calculate the robot's position in the `map` frame.
*   **`move2aruco`**: A simple controller that subscribes to marker poses and sends velocity commands to drive the robot towards them.

## Future Work

*   **Obstacle Detection:** Implement a CNN model to override movement commands when an obstacle is detected.
*   **Extended Kalman Filter (EKF):** Fuse wheel odometry with visual localization for more robust tracking during marker occlusions.
*   **Waypoint Navigation:** Implement a path planner to navigate between known markers.

## Acknowledgments

*   **OpenCV:** For the ArUco library.
*   **NVIDIA-AI-IOT:** For the base JetBot ROS support.
