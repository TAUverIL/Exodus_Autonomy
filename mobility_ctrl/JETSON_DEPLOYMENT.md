# Jetson Deployment Guide - 3-Camera Navigation System

---

## 🎯 SESSION PROGRESS TRACKER

**Last Updated:** 2026-01-07
**Current Phase:** Phase 1 - Verification (Step 5: Initial Testing)
**Current Status:** Ready to test RTAB-Map SLAM and costmap integration

### ✅ Completed Tasks
- [x] RTAB-Map configuration for 3 ZED2i cameras
- [x] RGBDImages interface configured (subscribe_rgbd: true, rgbd_cameras: 0)
- [x] Nav2 costmap configuration files created
- [x] Launch files created (multi_zed_rtab.launch.py, real_nav.launch.py)
- [x] Camera synchronization fixed (approx_sync enabled)
- [x] RGB image publishing enabled for cameras

### 🚧 Current Task: SYSTEM VERIFICATION
**YOU ARE HERE →** Need to run initial tests to verify everything works

### ⏭️ Next Immediate Action
Run these commands on Jetson to test the system:

```bash
# Terminal 1: Start RTAB-Map + Cameras
cd ~/Exodus2025
source install/setup.bash
ros2 launch multi_zed_rtab multi_zed_rtab.launch.py

# Terminal 2: Verify topics (in new terminal)
ros2 topic hz /rtabmap/grid_map  # Should be ~1 Hz
ros2 topic hz /zed_multi/camera1/point_cloud/cloud_registered

# Terminal 3: Start Costmaps (after RTAB-Map is running)
ros2 launch multi_zed_rtab real_nav.launch.py

# Terminal 4: Verify costmaps
ros2 topic hz /global_costmap/costmap  # Should be ~1 Hz
ros2 topic hz /local_costmap/costmap   # Should be ~2 Hz
```

### 📋 Verification Checklist (Phase 1)
- [ ] All 3 cameras detected and publishing point clouds at ~15 Hz
- [ ] `/rtabmap/grid_map` publishing at 1 Hz (CRITICAL!)
- [ ] `/global_costmap/costmap` publishing at 1 Hz
- [ ] `/local_costmap/costmap` publishing at 2 Hz
- [ ] RViz2 visualization shows maps correctly
- [ ] RQT graph shows correct topic connections

### 🎯 After Verification Passes - Next Steps
1. **Week 1**: Integrate Stanley controller with Nav2
2. **Week 2**: Add Nav2 planner and test waypoint navigation
3. **Week 3**: Full autonomous navigation mission testing

---

## Quick Reference
This guide covers deploying the updated 3-camera RTAB-Map + Nav2 costmap integration to your NVIDIA Jetson.

---

## Step 1: Pull Changes to Jetson

### On Your Development Machine (Windows)
```bash
cd C:\Users\Maynan\Desktop\Exodus_Autonomy

# Stage all changes
git add .

# Commit with descriptive message
git commit -m "Add 3-camera Nav2 costmap integration

- Update RTAB-Map config for 3 ZED2i cameras with grid map publishing
- Create Nav2 costmap configuration (local + global)
- Add costmap launch file connecting RTAB-Map to navigation
- Update camera URDF for camera3 (manipulator mount)
- Add camera3 serial: 33289078"

# Push to repository
git push origin main
```

### On Your Jetson
```bash
# Navigate to your workspace
cd ~/ros2_ws/src/Exodus_Autonomy/mobility_ctrl  # Adjust path as needed

# Pull latest changes
git pull origin main

# You should see:
# - config/costmap_params.yaml (NEW)
# - launch/costmap_bringup.launch.py (NEW)
# - NAVIGATION_SETUP.md (NEW)
# - JETSON_DEPLOYMENT.md (NEW)
# - config/multi_camera_config.yaml (MODIFIED)
# - src/nav2_stanley/urdf/zed_cameras.xacro.urdf (MODIFIED)
```

---

## Step 2: Verify Dependencies

### Required ROS 2 Packages
```bash
# Check if Nav2 costmap is installed
ros2 pkg list | grep nav2_costmap_2d

# If missing, install Nav2
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup

# Check lifecycle manager
ros2 pkg list | grep nav2_lifecycle_manager

# Check ZED ROS2 wrapper
ros2 pkg list | grep zed_wrapper

# Check RTAB-Map
ros2 pkg list | grep rtabmap_ros
```

### Install Missing Dependencies
```bash
# If any packages are missing:
sudo apt install \
  ros-humble-nav2-costmap-2d \
  ros-humble-nav2-lifecycle-manager \
  ros-humble-rtabmap-ros \
  ros-humble-zed-wrapper
```

---

## Step 3: Rebuild Workspace

```bash
# Navigate to workspace root
cd ~/ros2_ws  # Adjust to your workspace path

# Clean previous builds (recommended)
rm -rf build/ install/ log/

# Build with dependencies
colcon build --symlink-install --packages-select mobility_ctrl

# Source the workspace
source install/setup.bash

# Add to .bashrc for automatic sourcing
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

---

## Step 4: Hardware Verification

### Check All 3 ZED Cameras
```bash
# List USB devices - should see 3 ZED cameras
lsusb | grep -i "2b03"

# Check ZED camera info
ros2 run zed_wrapper zed_camera_info

# Expected output:
# Camera 1: SN 32514439 - ZED 2i
# Camera 2: SN 31718458 - ZED 2i
# Camera 3: SN 33289078 - ZED 2i
```

### Verify Camera Permissions
```bash
# Add user to video group (if not already)
sudo usermod -aG video $USER

# Apply udev rules for ZED
wget https://github.com/stereolabs/zed-ros2-wrapper/raw/master/zed_wrapper/udev/99-zed.rules
sudo cp 99-zed.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger

# Reboot if needed
sudo reboot
```

---

## Step 5: Initial Testing

### Test 1: Launch Cameras + RTAB-Map
```bash
# Source workspace
source ~/ros2_ws/install/setup.bash

# Launch multi-camera RTAB-Map
ros2 launch mobility_ctrl multi_camera_launch.py

# Expected output:
# [INFO] [zed_wrapper_camera1]: ZED SDK Version: X.X.X
# [INFO] [zed_wrapper_camera2]: ZED SDK Version: X.X.X
# [INFO] [zed_wrapper_camera3]: ZED SDK Version: X.X.X
# [INFO] [rtabmap]: RTAB-Map initialized
```

**In a New Terminal:**
```bash
# Check if all camera point clouds are publishing
ros2 topic hz /zed_front/zed_node/point_cloud/cloud_registered
# Expected: ~15 Hz

ros2 topic hz /zed_rear/zed_node/point_cloud/cloud_registered
# Expected: ~15 Hz

ros2 topic hz /zed_manipulator/zed_node/point_cloud/cloud_registered
# Expected: ~15 Hz

# CRITICAL: Check if RTAB-Map is publishing grid map
ros2 topic hz /rtabmap/grid_map
# Expected: ~1 Hz
# If this shows nothing, RTAB-Map grid generation is not working!
```

### Test 2: Launch Costmaps (Keep RTAB-Map Running)
```bash
# In a new terminal
source ~/ros2_ws/install/setup.bash

ros2 launch mobility_ctrl costmap_bringup.launch.py

# Expected output:
# [INFO] [global_costmap]: Configuring
# [INFO] [local_costmap]: Configuring
# [INFO] [costmap_lifecycle_manager]: Activating global_costmap
# [INFO] [costmap_lifecycle_manager]: Activating local_costmap
```

**Verify Costmap Topics:**
```bash
# Check costmap publishing
ros2 topic hz /global_costmap/costmap
# Expected: ~1 Hz

ros2 topic hz /local_costmap/costmap
# Expected: ~2 Hz

ros2 topic hz /local_costmap/voxel_grid
# Expected: ~2 Hz
```

### Test 3: Visualize in RViz2
```bash
# Launch RViz2
ros2 run rviz2 rviz2

# Add displays:
# 1. TF - to see coordinate frames
# 2. Map → Topic: /rtabmap/grid_map → Shows RTAB-Map 2D grid
# 3. Map → Topic: /global_costmap/costmap → Shows global costmap
# 4. Map → Topic: /local_costmap/costmap → Shows local costmap
# 5. PointCloud2 → Topic: /zed_front/.../cloud_registered
# 6. PointCloud2 → Topic: /local_costmap/voxel_grid → Shows 3D obstacles

# Set Fixed Frame: map
```

### Test 4: Check RQT Graph
```bash
# Install rqt_graph if needed
sudo apt install ros-humble-rqt-graph

# Launch rqt_graph
rqt_graph

# Expected connections (toggle "Hide Dead Sinks"):
# - rtabmap → /rtabmap/grid_map → global_costmap
# - zed_wrapper_camera1 → point_cloud → local_costmap
# - zed_wrapper_camera2 → point_cloud → local_costmap
# - zed_wrapper_camera3 → point_cloud → local_costmap
```

---

## Step 6: Troubleshooting Common Issues

### Issue: Camera Not Detected
```bash
# Check USB connections
lsusb | grep -i "2b03"

# Check dmesg for USB errors
dmesg | grep -i zed

# Verify camera serial in config matches physical camera
cat config/multi_camera_config.yaml | grep cam_serials
```

### Issue: RTAB-Map Not Publishing Grid Map
```bash
# Check if Grid/FromDepth is enabled
ros2 param get /rtabmap/rtabmap Grid/FromDepth
# Should return: true

# If false, check config file
cat config/multi_camera_config.yaml | grep "Grid/FromDepth"

# Restart RTAB-Map after verifying config
```

### Issue: Costmap Shows Only Unknown Cells
```bash
# Check if RTAB-Map grid is publishing
ros2 topic echo /rtabmap/grid_map --once

# Check if global_costmap is subscribed to correct topic
ros2 topic info /rtabmap/grid_map
# Should show global_costmap as a subscriber
```

### Issue: High CPU/Memory Usage
```bash
# Monitor resources
htop

# If CPU is maxed out, reduce point cloud rate
# Edit config/multi_camera_config.yaml:
# point_cloud_freq: 10.0  # Reduce from 15.0

# Or increase decimation
# cloud_decimation: 8  # Increase from 4
```

### Issue: TF Transform Errors
```bash
# View TF tree
ros2 run tf2_tools view_frames

# Check for missing transforms
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo base_link camera3_camera_link

# Verify URDF is loaded
ros2 param get /robot_state_publisher robot_description
```

---

## Step 7: Performance Optimization for Jetson

### Jetson Power Mode
```bash
# Check current power mode
sudo nvpmodel -q

# Set to maximum performance (Mode 0)
sudo nvpmodel -m 0

# Enable jetson_clocks for maximum frequency
sudo jetson_clocks

# Verify
sudo nvpmodel -q
```

### Memory Management
```bash
# Check available memory
free -h

# If low memory, create swap file (if not exists)
sudo fallocate -l 4G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile

# Make permanent
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
```

### Optimize Point Cloud Processing
Edit `config/multi_camera_config.yaml`:
```yaml
# Reduce computational load
depth:
  point_cloud_freq: 10.0  # From 15.0 Hz

cloud_decimation: 6  # From 4 (more aggressive downsampling)
cloud_voxel_size: 0.08  # From 0.05 (larger voxels)
```

---

## Step 8: Create System Launch File (Recommended)

Create a single launch file to start everything:

**File:** `launch/full_navigation_bringup.launch.py`
```python
#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    # Include multi-camera + RTAB-Map launch
    multi_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('mobility_ctrl'),
                'launch',
                'multi_camera_launch.py'
            ])
        )
    )

    # Include costmap launch
    costmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('mobility_ctrl'),
                'launch',
                'costmap_bringup.launch.py'
            ])
        )
    )

    return LaunchDescription([
        multi_camera_launch,
        costmap_launch,
    ])
```

**Usage:**
```bash
# Single command to launch everything
ros2 launch mobility_ctrl full_navigation_bringup.launch.py
```

---

## What to Do Next - Roadmap to Full Autonomy

### Phase 1: Verification (DO THIS FIRST)
- [ ] All 3 cameras detected and publishing point clouds
- [ ] `/rtabmap/grid_map` publishing at 1 Hz
- [ ] Global costmap receiving RTAB-Map grid
- [ ] Local costmap showing obstacles from cameras
- [ ] RQT graph shows all connections

### Phase 2: Navigation Stack Integration
- [ ] **Install Nav2 Controllers**
  ```bash
  sudo apt install ros-humble-nav2-controller ros-humble-nav2-planner
  ```

- [ ] **Configure Path Planner** (NavFn or Smac Planner)
  - Create `config/planner_params.yaml`
  - Configure for Ackermann constraints

- [ ] **Configure Controller** (You already have Stanley controller!)
  - Integrate with Nav2 controller server
  - Configure for differential/Ackermann drive

- [ ] **Add Behavior Server**
  ```bash
  sudo apt install ros-humble-nav2-behaviors
  ```
  - Configure recovery behaviors (backup, spin, wait)

### Phase 3: Mission Planning Integration
- [ ] **GPS Waypoint Navigation**
  - Integrate GPS → map frame conversion
  - Create waypoint following mission

- [ ] **ArUco Marker Detection**
  - Integrate existing YOLO detector with costmap
  - Add ArUco detection for ERC tasks
  - Create marker approach behavior

- [ ] **YOLO Object Detection → Costmap**
  - Add semantic layer to costmap
  - Mark detected objects as obstacles or goals

### Phase 4: ERC-Specific Features
- [ ] **Blind Teleoperation Mode**
  - Configure costmap for safe teleoperation
  - Add virtual joystick/gamepad interface

- [ ] **Sample Collection Assistance**
  - Use camera3 (manipulator) for close-range guidance
  - Visual servoing for sample alignment

- [ ] **Autonomous Navigation Mission**
  - GPS waypoint following
  - ArUco marker detection and approach
  - Obstacle avoidance
  - Return to start

---

## Next Immediate Steps (In Order)

### 1. Test Current System (Today)
```bash
# On Jetson
cd ~/ros2_ws/src/Exodus_Autonomy/mobility_ctrl
git pull origin main
cd ~/ros2_ws
colcon build --symlink-install --packages-select mobility_ctrl
source install/setup.bash

# Launch and verify
ros2 launch mobility_ctrl multi_camera_launch.py
# In new terminal:
ros2 launch mobility_ctrl costmap_bringup.launch.py

# Check everything works
ros2 topic hz /rtabmap/grid_map
ros2 topic hz /global_costmap/costmap
ros2 topic hz /local_costmap/costmap
```

### 2. Add Nav2 Planner (Tomorrow)
```bash
# Install planner
sudo apt install ros-humble-nav2-planner

# Create planner configuration
# File: config/planner_params.yaml
```

### 3. Integrate Stanley Controller (This Week)
- Connect your existing Stanley controller to Nav2
- Test path following with generated paths

### 4. Test Autonomous Navigation (Next Week)
- Define test waypoints
- Run full navigation mission
- Tune parameters based on performance

---

## Quick Command Reference

### Daily Startup Sequence
```bash
# 1. SSH to Jetson
ssh username@jetson-ip

# 2. Set max performance
sudo nvpmodel -m 0 && sudo jetson_clocks

# 3. Source workspace
source ~/ros2_ws/install/setup.bash

# 4. Launch navigation stack
ros2 launch mobility_ctrl full_navigation_bringup.launch.py
```

### Monitoring Commands
```bash
# Check all topics
ros2 topic list

# Monitor critical topics
watch -n 1 'ros2 topic hz /rtabmap/grid_map'

# Check node status
ros2 node list

# Monitor resources
htop

# View logs
ros2 run rqt_console rqt_console
```

---

## Support and Documentation

- **Navigation Setup:** See `NAVIGATION_SETUP.md`
- **Nav2 Docs:** https://navigation.ros.org
- **RTAB-Map Docs:** https://github.com/introlab/rtabmap_ros
- **ZED ROS2 Docs:** https://www.stereolabs.com/docs/ros2

**Good luck with ERC 2025!** 🚀

---

**Document Last Updated:** 2026-01-07
**Camera Serial Numbers:**
- Camera 1 (Front): 32514439
- Camera 2 (Rear): 31718458
- Camera 3 (Manipulator): 33289078

---

## 📝 How to Update Progress

Each time you complete a task or test, update the SESSION PROGRESS TRACKER section at the top:
1. Mark completed items with [x]
2. Update "Current Task" to reflect what you're working on
3. Update "Next Immediate Action" with the next command to run
4. Update "Last Updated" date
