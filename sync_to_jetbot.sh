#!/bin/bash
#
# Sync workspace to JetBot for hardware testing
#
# This script copies the ROS workspace to the JetBot and rebuilds it there.
# Modify the JETBOT_IP and JETBOT_USER variables to match your setup.
#
# Usage:
#   ./sync_to_jetbot.sh              # Full sync and rebuild
#   ./sync_to_jetbot.sh --quick      # Quick sync (no rebuild)
#   ./sync_to_jetbot.sh --code-only  # Sync only Python code (fastest)
#

set -e  # Exit on error

# JetBot configuration
JETBOT_IP="${JETBOT_IP:-jetbot.local}"  # Can also be IP like 192.168.1.100
JETBOT_USER="${JETBOT_USER:-jetbot}"
JETBOT_WORKSPACE="${JETBOT_WORKSPACE:-~/mobrob/jetbot-backup/ws}"

# Local workspace
LOCAL_WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Functions
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

print_warning() {
    echo -e "${YELLOW}⚠${NC} $1"
}

print_info() {
    echo -e "${BLUE}ℹ${NC} $1"
}

# Check if JetBot is reachable
check_connection() {
    print_header "Checking JetBot Connection"
    print_info "Testing connection to ${JETBOT_USER}@${JETBOT_IP}..."
    
    if ssh -o ConnectTimeout=5 -o BatchMode=yes "${JETBOT_USER}@${JETBOT_IP}" echo "Connection successful" 2>/dev/null; then
        print_success "Connected to JetBot"
        return 0
    else
        print_error "Cannot connect to JetBot at ${JETBOT_IP}"
        print_info "Make sure:"
        print_info "  1. JetBot is powered on"
        print_info "  2. You can SSH to it: ssh ${JETBOT_USER}@${JETBOT_IP}"
        print_info "  3. SSH keys are set up (or use password authentication)"
        print_info ""
        print_info "You can set custom values:"
        print_info "  export JETBOT_IP=192.168.1.100"
        print_info "  export JETBOT_USER=your_username"
        exit 1
    fi
}

# Sync files
sync_files() {
    local mode=$1
    print_header "Syncing Files to JetBot"
    
    # Create remote directory if it doesn't exist
    ssh "${JETBOT_USER}@${JETBOT_IP}" "mkdir -p ${JETBOT_WORKSPACE}"
    
    if [ "$mode" == "code-only" ]; then
        print_info "Quick sync: Python code only"
        rsync -avz --delete \
            --include='*.py' \
            --include='*.yaml' \
            --include='*/' \
            --exclude='*' \
            "${LOCAL_WORKSPACE}/src/py_pubsub/" \
            "${JETBOT_USER}@${JETBOT_IP}:${JETBOT_WORKSPACE}/src/py_pubsub/"
    else
        print_info "Full sync: entire workspace"
        rsync -avz --delete \
            --exclude='build/' \
            --exclude='install/' \
            --exclude='log/' \
            --exclude='*.pyc' \
            --exclude='__pycache__/' \
            "${LOCAL_WORKSPACE}/" \
            "${JETBOT_USER}@${JETBOT_IP}:${JETBOT_WORKSPACE}/"
    fi
    
    print_success "Files synced successfully"
}

# Build workspace on JetBot
build_workspace() {
    print_header "Building Workspace on JetBot"
    
    print_info "Running colcon build..."
    ssh "${JETBOT_USER}@${JETBOT_IP}" << 'ENDSSH'
cd ~/mobrob/jetbot-backup/ws
source /opt/ros/humble/setup.bash  # Adjust if using different ROS distro
colcon build --packages-select py_pubsub --symlink-install
echo "Build complete!"
ENDSSH
    
    print_success "Build completed"
}

# Display usage instructions
show_instructions() {
    print_header "Deployment Complete!"
    
    echo -e "To run the localization system on JetBot:"
    echo -e ""
    echo -e "${GREEN}1. SSH into JetBot:${NC}"
    echo -e "   ssh ${JETBOT_USER}@${JETBOT_IP}"
    echo -e ""
    echo -e "${GREEN}2. Source the workspace:${NC}"
    echo -e "   cd ${JETBOT_WORKSPACE}"
    echo -e "   source install/setup.bash"
    echo -e ""
    echo -e "${GREEN}3. Launch the system:${NC}"
    echo -e "   ${YELLOW}# For testing with real camera (needs aruco_opencv):${NC}"
    echo -e "   ros2 launch py_pubsub aruco_localization.launch.xml use_simulator:=false"
    echo -e ""
    echo -e "   ${YELLOW}# Just the localization node (if you have detections):${NC}"
    echo -e "   ros2 run py_pubsub aruco_localizer"
    echo -e ""
    echo -e "${GREEN}4. Monitor the system (in another terminal):${NC}"
    echo -e "   ros2 run py_pubsub test_localization"
    echo -e ""
    echo -e "${GREEN}5. Check TF tree:${NC}"
    echo -e "   ros2 run tf2_ros tf2_echo map base_link"
    echo -e ""
    print_warning "Remember to update marker_map.yaml with your actual marker positions!"
}

# Main script
main() {
    local mode="full"
    
    # Parse arguments
    case "${1:-}" in
        --quick)
            mode="quick"
            ;;
        --code-only)
            mode="code-only"
            ;;
        --help)
            echo "Usage: $0 [--quick|--code-only|--help]"
            echo ""
            echo "Options:"
            echo "  --quick      Sync files but don't rebuild"
            echo "  --code-only  Sync only Python/YAML files (fastest)"
            echo "  --help       Show this help message"
            exit 0
            ;;
    esac
    
    print_header "JetBot Deployment Script"
    print_info "Local workspace: ${LOCAL_WORKSPACE}"
    print_info "Target: ${JETBOT_USER}@${JETBOT_IP}:${JETBOT_WORKSPACE}"
    print_info "Mode: ${mode}"
    
    check_connection
    sync_files "$mode"
    
    if [ "$mode" != "quick" ] && [ "$mode" != "code-only" ]; then
        build_workspace
    else
        print_warning "Skipping build (use full sync if you need to rebuild)"
    fi
    
    show_instructions
}

# Run main function
main "$@"
