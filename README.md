Team: Israk Arafat, Shuvra Smaran Das

Problem & Why: Use a sequence of ARUCO markers to track the robot's position at all times (SLAM-ish) and implement in ROS. On this project, we can utilize both ROS and the math we learned in class regarding rotation and translation. 

Ambitious plan: We extend the project by adding an obstacle detector that runs directly on the Jetson Nano. A small CNN takes the JetBot’s forward-facing image and outputs a simple binary decision: safe or obstacle-ahead. This node runs in real time, and if the network signals an obstacle, the controller overrides the user’s forward command and stops the robot before contact. This gives us a minimal collision-avoidance layer that integrates with the ArUco-based localization.

Knowledge Have: Linear algebra for rotations and translations, Python, basic ROS, and OpenCV.

Need to learn: Camera calibration, ArUco detection, ROS TF/frames, RViz.

Importance: Demonstrates reliable, explainable localization with only a camera. And we can do navigation tasks without expensive sensors.

Related Work: 
OpenCV ArUco and Aruco_ros - Detect the ARUCO markers and estimate pose. (use directly)
Fiducials - Learn how to determine the robot’s position and orientation from multiple ARUCO markers
Robot_localization - EKF - fuse fiducial pose with odom (use directly)

Simplest Solution: For the simplest baseline, we’ll calibrate the camera, place a single ArUco marker with a known world pose, estimate the robot pose from that marker detection, and publish map -> base_link on tf. Localization will temporarily drop out whenever the tag is out of view.

Plan and Division of Work:
By the second report:
Israk: camera calibration, ArUco detection, compute & publish /tf, RViz
Shuvra: static TF, simple odom publisher, basic evaluation script
Our safe goal is single-marker localization at ≥10 Hz. Moderate is multi-marker fusion with an /odom publisher. An ambitious plan is an initial EKF fusing wheel.

By the final report:
Israk: robust multi-marker fusion, map file loader.
Shuvra: EKF/UKF integration, waypoint demo.
By the final report, our safe goal is stable room-scale localization despite brief occlusions; moderate is waypoint following using the tag-based pose; ambitious is auto-building and updating the marker map.
Extended goal: train a CNN model with a dataset captured by the JetBot to detect obstacles and override the command of the user. 

Fallback plan: We can skip SLAM and use a static map. We can reduce FPS/resolution. If not possible to train the model, we will try to use a pretrained model. 

Measurable and Quantifiable Goals:
Detection rate: By the second report, we plan to have >5 HZ, and by the final report, we plan to have > 15HZ. 
Pose error: By the final report, we plan to have <0.2 m RMS. 
YAML marker-map loader with schema test.
Obstacle classifier: Achieve a stable binary obstacle-ahead prediction at ≥5 Hz on the Jetson Nano, with at least 90 percent correct stop decisions in a simple hallway test.

---
