#!/bin/bash
# ============================================
# Environment Setup Script for AA-Evasion Drone Project
# ============================================

set -e

echo "=========================================="
echo "Setting up AA-Evasion Drone Environment"
echo "=========================================="

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Check if ROS2 Humble is installed
if [ ! -f /opt/ros/humble/setup.bash ]; then
    echo -e "${RED}Error: ROS2 Humble not found!${NC}"
    echo "Please install ROS2 Humble first:"
    echo "https://docs.ros.org/en/humble/Installation.html"
    exit 1
fi

echo -e "${GREEN}✓ ROS2 Humble found${NC}"

# Check if Gazebo is installed
if ! command -v gazebo &> /dev/null; then
    echo -e "${YELLOW}Warning: Gazebo not found!${NC}"
    echo "Installing Gazebo Classic 11..."
    sudo apt update
    sudo apt install -y gazebo libgazebo-dev
fi

echo -e "${GREEN}✓ Gazebo found${NC}"

# Install Python dependencies
echo ""
echo "Installing Python dependencies..."
if [ -f requirements.txt ]; then
    pip3 install -r requirements.txt
    echo -e "${GREEN}✓ Python dependencies installed${NC}"
else
    echo -e "${YELLOW}Warning: requirements.txt not found${NC}"
fi

# Install vcstool if not present
if ! command -v vcs &> /dev/null; then
    echo ""
    echo "Installing vcstool..."
    sudo apt install -y python3-vcstool
fi

# Clone external ROS2 packages
echo ""
echo "Cloning external ROS2 packages..."
if [ -f dependencies.repos ]; then
    vcs import < dependencies.repos
    echo -e "${GREEN}✓ External packages cloned${NC}"
else
    echo -e "${YELLOW}Warning: dependencies.repos not found${NC}"
fi

# Install rosdep dependencies
echo ""
echo "Installing ROS2 package dependencies..."
cd ros2_ws
sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y
cd ..

# Build ROS2 workspace
echo ""
echo "Building ROS2 workspace..."
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
cd ..

echo ""
echo -e "${GREEN}=========================================="
echo "Environment setup complete!"
echo "==========================================${NC}"
echo ""
echo "To use the workspace, run:"
echo "  source ros2_ws/install/setup.bash"
echo ""
echo "Or add to your ~/.bashrc or ~/.zshrc:"
echo "  source ~/workspace/air_defense_evasion_drone_project/ros2_ws/install/setup.bash"
