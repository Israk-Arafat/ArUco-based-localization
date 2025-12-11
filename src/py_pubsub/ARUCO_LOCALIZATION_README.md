# ArUco-Based Robot Localization System

**ECE 417 Mobile Robotics Project**

This project implements SLAM-like localization using ArUco markers with ROS2. The system tracks the robot's position by detecting fiducial markers with known positions in the environment.

## 🎯 Project Goals

### Checkpoint 1 (Second Report)
- ✅ Camera calibration
- ✅ ArUco marker detection
- ✅ TF frame publishing (map → base_link)
- ✅ RViz visualization
- 🎯 **Target**: >5 Hz detection rate

### Final Report
- 🔄 Multi-marker fusion for robust localization
- 🔄 EKF/UKF integration with wheel odometry
- 🔄 Waypoint navigation demo
- 🎯 **Target**: >15 Hz detection, <0.2m RMS pose error

### Stretch Goals
- 🚀 CNN-based obstacle detection on Jetson Nano
- 🚀 Collision avoidance override
- 🚀 Auto-mapping of marker positions

---

## 📁 Project Structure

```
jetbot-backup/ws/src/py_pubsub/
├── config/
│   └── marker_map.yaml          # Known marker positions
├── launch/
│   └── aruco_localization.launch.xml  # Launch file
├── py_pubsub/
│   ├── aruco_localization.py    # Main localization node
│   ├── marker_tf_publisher.py   # Static marker TF broadcaster
│   ├── simulator.py             # 2D simulator for testing
│   └── test_localization.py     # Monitoring/testing tool
└── sync_to_jetbot.sh            # Deployment script
```

---

## 🚀 Quick Start

### 1. Build the Workspace

```bash
cd ~/mobrob/jetbot-backup/ws
source /opt/ros/humble/setup.bash
colcon build --packages-select py_pubsub --symlink-install
source install/setup.bash
```

### 2. Test with Simulator (No Hardware Needed)

```bash
# Terminal 1: Launch the full system with simulator
ros2 launch py_pubsub aruco_localization.launch.xml

# Terminal 2: Monitor the system
ros2 run py_pubsub test_localization

# Terminal 3: Send motion commands (optional)
ros2 topic pub /jetbot_cmdvel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"
```

The simulator will show a matplotlib window with the robot and markers. The localization system should detect markers and publish the robot's pose.

### 3. Visualize in RViz

```bash
rviz2
```

Add the following displays:
- **TF**: Shows coordinate frames (map, base_link, camera_link, marker_X)
- **Pose**: Subscribe to `/robot_pose` to see estimated position
- Set **Fixed Frame** to `map`

---

## 🤖 Deploy to JetBot

### Prerequisites
1. JetBot is on and connected to network
2. You can SSH to it: `ssh jetbot@jetbot.local`
3. ROS2 is installed on JetBot

### Deploy

```bash
cd ~/mobrob/jetbot-backup/ws

# Full sync and build
./sync_to_jetbot.sh

# Quick sync (Python files only, no rebuild)
./sync_to_jetbot.sh --code-only
```

### Run on JetBot

```bash
# SSH into JetBot
ssh jetbot@jetbot.local

# Navigate and source workspace
cd ~/mobrob/jetbot-backup/ws
source install/setup.bash

# Start localization (requires aruco_opencv for real camera)
ros2 launch py_pubsub aruco_localization.launch.xml use_simulator:=false

# Or run just the localizer (if you have /aruco_detections from elsewhere)
ros2 run py_pubsub aruco_localizer
```

---

## 📊 System Architecture

```
┌─────────────────┐
│  Camera Node    │  (aruco_opencv or simulator)
│  (CSI Camera)   │
└────────┬────────┘
         │ /aruco_detections
         ▼
┌─────────────────┐     ┌──────────────────┐
│ ArUco Localizer │────▶│  TF Broadcaster  │
│                 │     │  map→base_link   │
└────────┬────────┘     └──────────────────┘
         │
         │ /robot_pose
         ▼
┌─────────────────┐
│  Navigation /   │
│  Controller     │
└─────────────────┘
```

**Static TF Tree:**
```
map
 ├── marker_0 (static)
 ├── marker_1 (static)
 ├── marker_2 (static)
 └── base_link (dynamic, from localization)
      └── camera_link (static offset)
```

---

## 🔧 Configuration

### Marker Map (`config/marker_map.yaml`)

Define your marker positions. Measure and record actual positions for your environment:

```yaml
markers:
  - id: 0
    position: [0.0, 0.0, 0.0]  # [x, y, z] in meters
    orientation: [0.0, 0.0, 0.0, 1.0]  # quaternion [x, y, z, w]
    description: "Origin marker"
  
  - id: 1
    position: [3.0, 0.0, 0.0]
    orientation: [0.0, 0.0, 0.0, 1.0]
    description: "3m forward"
```

**Tips:**
- Choose a coordinate system origin (e.g., room corner)
- Measure marker positions relative to origin
- Use orientation for markers on different walls
- Print markers at least 10-20cm size for 1-3m distances

### Camera-to-Base Transform

Edit the static transform in `aruco_localization.launch.xml`:

```xml
<!-- Adjust these values based on your robot's camera mount -->
<node pkg="tf2_ros" exec="static_transform_publisher" name="camera_to_base" 
      args="0.1 0 0.15 0 0 0 base_link camera_link" output="screen"/>
```

Format: `x y z roll pitch yaw parent_frame child_frame`

---

## 📈 Testing & Evaluation

### Monitor Performance

```bash
ros2 run py_pubsub test_localization
```

This displays:
- Detection rate (Hz)
- Pose update rate (Hz)
- Current detected markers
- Robot pose estimate
- TF tree status

### Check Specific Topics

```bash
# View detections
ros2 topic echo /aruco_detections

# View pose estimates
ros2 topic echo /robot_pose

# Check TF transform
ros2 run tf2_ros tf2_echo map base_link

# List all TFs
ros2 run tf2_tools view_frames
```

### Measure Localization Error

1. Place robot at known position
2. Record estimated position from `/robot_pose`
3. Calculate error: `sqrt((x_est - x_true)^2 + (y_est - y_true)^2)`
4. Repeat at multiple positions for RMS error

---

## 🐛 Troubleshooting

### No markers detected
- ✅ Check camera is working: `ros2 topic list` (look for camera topics)
- ✅ Verify markers are in view and properly lit
- ✅ Check marker dictionary matches (default: DICT_4X4_50)
- ✅ Print larger markers if detection distance is too far

### Low detection rate (<5 Hz)
- ✅ Check camera FPS settings
- ✅ Reduce image resolution if using high-res camera
- ✅ Verify Jetson Nano isn't throttling (check temperature)

### Incorrect pose estimates
- ✅ Verify marker positions in `marker_map.yaml` are accurate
- ✅ Check camera calibration parameters
- ✅ Verify camera-to-base_link transform is correct
- ✅ Ensure markers are flat and not warped

### TF errors
- ✅ Check all nodes are running: `ros2 node list`
- ✅ Verify TF tree: `ros2 run tf2_tools view_frames`
- ✅ Check for timing issues: use `ros2 run tf2_ros tf2_monitor`

---

## 📚 Related Work & References

- **OpenCV ArUco**: [docs.opencv.org/aruco](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html)
- **aruco_ros**: ROS wrapper for ArUco detection
- **Fiducials**: [wiki.ros.org/fiducials](http://wiki.ros.org/fiducials)
- **robot_localization**: EKF package for sensor fusion

---

## 🎓 Learning Resources

### Camera Calibration
- [OpenCV Camera Calibration Tutorial](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html)
- Use checkerboard pattern and `cv2.calibrateCamera()`

### ROS TF
- [TF2 Tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html)
- Understanding coordinate frame relationships

### ArUco Markers
- [ArUco Marker Generation](https://chev.me/arucogen/)
- Print at scale, mount on rigid surface

---

## 📝 Team Division

**Israk:**
- Camera calibration
- ArUco detection setup
- TF publishing
- RViz configuration
- Multi-marker fusion (final)
- Map file loader (final)

**Shuvra:**
- Static TF setup
- Odometry publisher
- Evaluation scripts
- EKF/UKF integration (final)
- Waypoint navigation demo (final)

---

## 🎯 Measurable Goals

| Metric | Second Report | Final Report |
|--------|--------------|--------------|
| Detection Rate | >5 Hz | >15 Hz |
| Pose Error | - | <0.2m RMS |
| Marker Map | 2-4 markers | 5+ markers |
| Obstacle Detection | - | 90% accuracy @ 5Hz |

---

## 🔮 Future Extensions

1. **Multi-Marker Fusion**: Weighted average or EKF
2. **Odometry Integration**: Fuse wheel encoders with visual localization
3. **Mapping**: Auto-detect and map unknown markers
4. **CNN Obstacle Avoidance**: Real-time on Jetson Nano
5. **Path Planning**: A* with ArUco waypoints

---

## 📄 License

Project for ECE 417 - Mobile Robotics

---

**Need help?** Check the troubleshooting section or run `test_localization` to diagnose issues.
