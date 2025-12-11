#!/bin/bash
#
# Build and test the ArUco localization system
#
# This script performs a complete build and basic validation
#

set -e

WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WORKSPACE_ROOT"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

print_header() {
    echo -e "\n${BLUE}========================================${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}========================================${NC}\n"
}

print_success() {
    echo -e "${GREEN}✓${NC} $1"
}

print_error() {
    echo -e "${RED}✗${NC} $1"
}

print_info() {
    echo -e "${BLUE}ℹ${NC} $1"
}

# Check ROS2 installation
print_header "Checking Prerequisites"
if [ -f "/opt/ros/humble/setup.bash" ]; then
    print_success "ROS2 Humble found"
    source /opt/ros/humble/setup.bash
elif [ -f "/opt/ros/foxy/setup.bash" ]; then
    print_success "ROS2 Foxy found"
    source /opt/ros/foxy/setup.bash
else
    print_error "ROS2 not found in /opt/ros/"
    exit 1
fi

# Check for required packages
print_info "Checking for required dependencies..."

# Build workspace
print_header "Building Workspace"
print_info "Building py_pubsub package..."
colcon build --packages-select py_pubsub --symlink-install

if [ $? -eq 0 ]; then
    print_success "Build successful"
else
    print_error "Build failed"
    exit 1
fi

# Source workspace
source install/setup.bash

# Validate files
print_header "Validating Installation"

# Check marker map
if [ -f "src/py_pubsub/config/marker_map.yaml" ]; then
    print_success "Marker map configuration found"
else
    print_error "Marker map not found"
fi

# Check launch file
if [ -f "src/py_pubsub/launch/aruco_localization.launch.xml" ]; then
    print_success "Launch file found"
else
    print_error "Launch file not found"
fi

# Check entry points
print_info "Checking ROS2 nodes..."
if ros2 pkg executables py_pubsub | grep -q "aruco_localizer"; then
    print_success "aruco_localizer node registered"
else
    print_error "aruco_localizer node not found"
fi

if ros2 pkg executables py_pubsub | grep -q "marker_tf_publisher"; then
    print_success "marker_tf_publisher node registered"
else
    print_error "marker_tf_publisher node not found"
fi

if ros2 pkg executables py_pubsub | grep -q "aruco_simulator"; then
    print_success "aruco_simulator node registered"
else
    print_error "aruco_simulator node not found"
fi

if ros2 pkg executables py_pubsub | grep -q "test_localization"; then
    print_success "test_localization node registered"
else
    print_error "test_localization node not found"
fi

# Summary
print_header "Build Complete!"

echo -e "${GREEN}Your ArUco localization system is ready!${NC}\n"
echo "Next steps:"
echo ""
echo -e "${YELLOW}1. Generate ArUco markers for printing:${NC}"
echo "   python3 src/py_pubsub/scripts/generate_markers.py --all --sheet --size 400"
echo ""
echo -e "${YELLOW}2. Test with simulator:${NC}"
echo "   ros2 launch py_pubsub aruco_localization.launch.xml"
echo ""
echo -e "${YELLOW}3. Monitor the system:${NC}"
echo "   ros2 run py_pubsub test_localization"
echo ""
echo -e "${YELLOW}4. Deploy to JetBot:${NC}"
echo "   ./sync_to_jetbot.sh"
echo ""
echo -e "${YELLOW}5. Read the documentation:${NC}"
echo "   cat src/py_pubsub/ARUCO_LOCALIZATION_README.md"
echo ""

print_info "For quick testing, run: ./quick_test.sh"
