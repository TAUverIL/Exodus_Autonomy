# Exodus Autonomy System - Complete Setup Guide

This document describes the complete autonomy system for the ERC 2025 rover with 3 ZED2i cameras.

## System Overview

The autonomy system consists of 4 main components:

1. **detector_3cam** - YOLO-based object detection with depth estimation for all 3 cameras
2. **grid_mapper** - Builds 2D occupancy grid from obstacle detections
3. **odom_fusion** - Fuses ZED visual odometry with wheel odometry using Kalman filter
4. **path_planner** - A* path planning from current pose to goal

## Files Created/Modified

### New Files Created

1. **autonomy_node/detector_3cam.py**
   - Subscribes to RGB + Depth from all 3 ZED cameras
   - Runs YOLOv8n on each camera's RGB stream
   - Estimates distance to each detected object using depth image
   - Publishes detections to `/autonomy/obstacle_detections`

2. **autonomy_node/grid_mapper_3cam.py**
   - Subscribes to obstacle detections from all cameras
   - Builds 2D occupancy grid (20m x 20m, 10cm resolution)
   - Inflates obstacles by 30cm for safety
   - Publishes to `/autonomy/occupancy_grid` for visualization and planning

3. **autonomy_node/odom_fusion.py**
   - Subscribes to `/rtabmap/odom` (ZED visual odometry)
   - Subscribes to `/wheel_odom` (from computer team)
   - Implements Kalman filter fusion
   - Publishes fused estimate to `/autonomy/fused_odom`

4. **autonomy_node/path_planner.py**
   - Subscribes to occupancy grid and fused odometry
   - Subscribes to goal pose from `/goal_pose`
   - Computes A* path avoiding obstacles
   - Publishes path to `/autonomy/waypoints` for computer team

5. **autonomy_node/launch/autonomy_system.launch.py**
   - Launches all 4 autonomy components together
   - Configurable parameters for grid size, resolution, etc.

### Modified Files

1. **multi_zed_rtab/launch/multi_zed_rtab.launch.py**
   - Added camera3 support (rgbd_sync3)
   - Updated rgbdx_sync to handle 3 cameras instead of 2
   - Changed `rgbd_cameras` parameter from 2 to 3

2. **autonomy_node/setup.py**
   - Added entry points for all 4 new nodes
   - Added launch file installation

3. **autonomy_node/package.xml**
   - Added dependencies: nav_msgs, geometry_msgs, std_msgs

## Topic Names and Data Flow

### Camera Topics (from ZED multi-camera launcher)
```
/camera1/zed_node_1/rgb/image_rect_color
/camera1/zed_node_1/depth/depth_registered
/camera2/zed_node_2/rgb/image_rect_color
/camera2/zed_node_2/depth/depth_registered
/camera3/zed_node_3/rgb/image_rect_color
/camera3/zed_node_3/depth/depth_registered
```

### Autonomy Topics (published by your nodes)
```
/autonomy/obstacle_detections     - ObstacleDetection (custom msg)
/autonomy/occupancy_grid          - OccupancyGrid
/autonomy/fused_odom              - Odometry
/autonomy/planned_path            - Path (for visualization)
/autonomy/waypoints               - Path (to computer team)
```

### Input Topics (from other teams)
```
/rtabmap/odom                     - Odometry (ZED visual odometry)
/wheel_odom                       - Odometry (from computer team)
/goal_pose                        - PoseStamped (goal position)
```

## Building the System

1. **Install dependencies** (if not already installed):
```bash
cd ~/Desktop/project/Exodus_Autonomy/mobility_ctrl
pip3 install ultralytics  # For YOLO
pip3 install opencv-python
pip3 install numpy
```

2. **Build the ROS2 packages**:
```bash
cd ~/Desktop/project/Exodus_Autonomy/mobility_ctrl
colcon build --packages-select autonomy_node multi_zed_rtab
source install/setup.bash
```

## Running the System

### Option 1: Run Everything Together

**Terminal 1** - Launch cameras + RTAB-Map:
```bash
source install/setup.bash
ros2 launch multi_zed_rtab multi_zed_rtab.launch.py
```

**Terminal 2** - Launch autonomy system:
```bash
source install/setup.bash
ros2 launch autonomy_node autonomy_system.launch.py
```

### Option 2: Run Components Individually (for debugging)

**Terminal 1** - Cameras + RTAB-Map:
```bash
ros2 launch multi_zed_rtab multi_zed_rtab.launch.py
```

**Terminal 2** - Detector:
```bash
ros2 run autonomy_node detector_3cam
```

**Terminal 3** - Grid Mapper:
```bash
ros2 run autonomy_node grid_mapper
```

**Terminal 4** - Odometry Fusion:
```bash
ros2 run autonomy_node odom_fusion
```

**Terminal 5** - Path Planner:
```bash
ros2 run autonomy_node path_planner
```

## Visualization in RViz2

1. **Open RViz2**:
```bash
rviz2
```

2. **Add the following displays**:
   - Fixed Frame: `base_link` or `odom`
   - Add → By Topic → `/autonomy/occupancy_grid` → Map
   - Add → By Topic → `/autonomy/planned_path` → Path
   - Add → By Topic → `/camera1/zed_node_1/rgb/image_rect_color` → Image
   - Add → By Topic → `/rtabmap/odom` → Odometry
   - Add → By Topic → `/autonomy/fused_odom` → Odometry

## Sending Goals

To send a goal position for path planning:

**Using command line**:
```bash
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'odom'},
  pose: {
    position: {x: 5.0, y: 2.0, z: 0.0},
    orientation: {w: 1.0}
  }
}"
```

**Using RViz2**:
1. Add "2D Nav Goal" tool
2. Click and drag on the map to set goal position

## Configuration Parameters

### Grid Mapper Parameters
Edit in `autonomy_system.launch.py`:
- `grid_resolution`: Cell size in meters (default: 0.1)
- `grid_width`: Grid width in meters (default: 20.0)
- `grid_height`: Grid height in meters (default: 20.0)
- `obstacle_radius`: Inflation radius in meters (default: 0.3)
- `decay_rate`: How fast old obstacles fade (default: 0.95)

## Troubleshooting

### Camera topics not appearing
- Check ZED cameras are connected and recognized
- Verify multi_zed_rtab.launch.py is running
- Use `ros2 topic list` to see available topics

### No detections published
- Ensure YOLO model downloaded (first run downloads yolov8n.pt)
- Check camera images with: `ros2 topic echo /camera1/zed_node_1/rgb/image_rect_color`
- Look for errors in detector_3cam terminal

### No path generated
- Verify occupancy grid is being published: `ros2 topic hz /autonomy/occupancy_grid`
- Check goal pose is set: `ros2 topic echo /goal_pose`
- Ensure current pose available: `ros2 topic echo /autonomy/fused_odom`
- Path may fail if goal is in obstacle or unreachable

### Wheel odometry not fusing
- Confirm computer team is publishing to `/wheel_odom`
- Check topic name matches their output
- Update `odom_fusion.py` line 40 if different topic name

## Camera Positioning Notes

The `grid_mapper_3cam.py` currently assumes:
- **camera1**: Front-facing camera
- **camera2**: Left-facing camera
- **camera3**: Right-facing camera

**IMPORTANT**: Update the camera positioning logic in `grid_mapper_3cam.py` lines 61-71 based on your actual camera mounting positions!

## Integration with Computer Team

Your system provides these outputs to the computer team:

1. **Waypoints**: `/autonomy/waypoints` (nav_msgs/Path)
   - List of poses from current position to goal
   - Computer team should subscribe and follow these waypoints

2. **Fused Odometry**: `/autonomy/fused_odom` (nav_msgs/Odometry)
   - Better pose estimate than either sensor alone
   - Computer team can use for their own navigation

Your system needs this input from computer team:

1. **Wheel Odometry**: `/wheel_odom` (nav_msgs/Odometry)
   - Adjust topic name in `odom_fusion.py` if needed

## Next Steps

1. **Test camera calibration**: Verify all 3 cameras are working
2. **Adjust camera positions**: Update grid_mapper based on actual mounting
3. **Tune parameters**: Adjust grid size, obstacle inflation, decay rate
4. **Test with real objects**: Run YOLO detection on real obstacles
5. **Coordinate with computer team**: Verify topic names and message formats
6. **Field testing**: Test complete pipeline in competition environment

## File Locations

All code is in: `C:\Users\Maynan\Desktop\project\Exodus_Autonomy\mobility_ctrl\src\autonomy_node\autonomy_node\`

- `detector_3cam.py`
- `grid_mapper_3cam.py`
- `odom_fusion.py`
- `path_planner.py`
