#!/bin/bash
# System Verification Script for Exodus Autonomy
# Run this to check if all components are working properly

echo "========================================="
echo "Exodus Autonomy System Verification"
echo "========================================="
echo ""

# Color codes
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

check_topic() {
    local topic=$1
    local name=$2

    if ros2 topic list | grep -q "^${topic}$"; then
        echo -e "${GREEN}✓${NC} ${name}: ${topic}"
        return 0
    else
        echo -e "${RED}✗${NC} ${name}: ${topic} (NOT FOUND)"
        return 1
    fi
}

check_node() {
    local node=$1
    local name=$2

    if ros2 node list | grep -q "${node}"; then
        echo -e "${GREEN}✓${NC} ${name} node is running"
        return 0
    else
        echo -e "${RED}✗${NC} ${name} node is NOT running"
        return 1
    fi
}

echo "1. Checking Camera Topics..."
echo "----------------------------"
check_topic "/camera1/zed_node_1/rgb/image_rect_color" "Camera 1 RGB"
check_topic "/camera1/zed_node_1/depth/depth_registered" "Camera 1 Depth"
check_topic "/camera2/zed_node_2/rgb/image_rect_color" "Camera 2 RGB"
check_topic "/camera2/zed_node_2/depth/depth_registered" "Camera 2 Depth"
check_topic "/camera3/zed_node_3/rgb/image_rect_color" "Camera 3 RGB"
check_topic "/camera3/zed_node_3/depth/depth_registered" "Camera 3 Depth"
echo ""

echo "2. Checking RTAB-Map Topics..."
echo "-------------------------------"
check_topic "/rtabmap/odom" "RTAB-Map Odometry"
check_topic "/camera1/rgbd_image" "Camera 1 RGBD Sync"
check_topic "/camera2/rgbd_image" "Camera 2 RGBD Sync"
check_topic "/camera3/rgbd_image" "Camera 3 RGBD Sync"
check_topic "/rgbd_images" "Multi-camera RGBD"
echo ""

echo "3. Checking Autonomy Nodes..."
echo "------------------------------"
check_node "detector_3cam" "3-Camera Detector"
check_node "grid_mapper" "Grid Mapper"
check_node "odom_fusion" "Odometry Fusion"
check_node "path_planner" "Path Planner"
echo ""

echo "4. Checking Autonomy Topics..."
echo "-------------------------------"
check_topic "/autonomy/obstacle_detections" "Obstacle Detections"
check_topic "/autonomy/occupancy_grid" "Occupancy Grid"
check_topic "/autonomy/fused_odom" "Fused Odometry"
echo ""

echo "5. Checking Topic Rates..."
echo "--------------------------"
echo "Camera 1 RGB rate:"
timeout 3 ros2 topic hz /camera1/zed_node_1/rgb/image_rect_color 2>/dev/null || echo -e "${YELLOW}  No data or timeout${NC}"

echo "Obstacle detections rate:"
timeout 3 ros2 topic hz /autonomy/obstacle_detections 2>/dev/null || echo -e "${YELLOW}  No data or timeout${NC}"

echo "Occupancy grid rate:"
timeout 3 ros2 topic hz /autonomy/occupancy_grid 2>/dev/null || echo -e "${YELLOW}  No data or timeout${NC}"
echo ""

echo "========================================="
echo "Verification Complete!"
echo "========================================="
echo ""
echo "Tip: If any checks failed, review AUTONOMY_SETUP.md for troubleshooting"
