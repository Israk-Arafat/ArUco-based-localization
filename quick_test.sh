#!/bin/bash
#
# Quick test script for ArUco localization system
# Builds, launches, and monitors the system in simulator mode
#

set -e

WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${BLUE}================================${NC}"
echo -e "${BLUE}ArUco Localization Quick Test${NC}"
echo -e "${BLUE}================================${NC}\n"

# Step 1: Source ROS
echo -e "${GREEN}[1/4]${NC} Sourcing ROS2..."
source /opt/ros/humble/setup.bash 2>/dev/null || {
    echo -e "${YELLOW}Warning: Could not source ROS2 from /opt/ros/humble${NC}"
    echo "Make sure ROS2 is installed"
}

# Step 2: Build
echo -e "\n${GREEN}[2/4]${NC} Building workspace..."
cd "$WORKSPACE_ROOT"
colcon build --packages-select py_pubsub --symlink-install

# Step 3: Source workspace
echo -e "\n${GREEN}[3/4]${NC} Sourcing workspace..."
source install/setup.bash

# Step 4: Check configuration
echo -e "\n${GREEN}[4/4]${NC} Checking configuration..."
if [ -f "$WORKSPACE_ROOT/src/py_pubsub/config/marker_map.yaml" ]; then
    echo -e "${GREEN}✓${NC} Marker map found"
else
    echo -e "${YELLOW}⚠${NC} Marker map not found at expected location"
fi

# Done
echo -e "\n${BLUE}================================${NC}"
echo -e "${GREEN}Setup Complete!${NC}"
echo -e "${BLUE}================================${NC}\n"

echo "You can now run the system:"
echo ""
echo -e "${YELLOW}Option 1: Full system with simulator${NC}"
echo "  ros2 launch py_pubsub aruco_localization.launch.xml"
echo ""
echo -e "${YELLOW}Option 2: Monitor in separate terminal${NC}"
echo "  ros2 run py_pubsub test_localization"
echo ""
echo -e "${YELLOW}Option 3: Manual control${NC}"
echo "  # Launch system first, then in another terminal:"
echo "  ros2 topic pub /jetbot_cmdvel geometry_msgs/msg/Twist \"{linear: {x: 0.3}, angular: {z: 0.0}}\""
echo ""
echo -e "${YELLOW}Option 4: Visualize in RViz${NC}"
echo "  rviz2"
echo "  # Add TF display and set Fixed Frame to 'map'"
echo ""

read -p "Launch the system now? (y/n) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo -e "\n${GREEN}Launching system...${NC}"
    echo "Press Ctrl+C to stop"
    echo ""
    ros2 launch py_pubsub aruco_localization.launch.xml
fi
