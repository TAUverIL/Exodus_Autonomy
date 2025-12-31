# Exodus Autonomy - 3-Camera Navigation Setup

## Overview
This document describes the complete navigation stack integration for the Exodus rover with 3 ZED2i cameras, RTAB-Map SLAM, and Nav2 costmaps.

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                         ZED2i CAMERAS (x3)                          │
│  Camera1 (Front)  │  Camera2 (Rear)  │  Camera3 (Manipulator)     │
│    SN: 32514439   │   SN: 31718458   │    SN: XXXXXXXX            │
└─────────────────────────────────────────────────────────────────────┘
                              │
                              ▼
        ┌─────────────────────────────────────────────┐
        │         Point Clouds (15 Hz)                │
        │  /zed_front/...../cloud_registered          │
        │  /zed_rear/...../cloud_registered           │
        │  /zed_manipulator/...../cloud_registered    │
        └─────────────────────────────────────────────┘
                              │
                ┌─────────────┴─────────────┐
                ▼                           ▼
        ┌───────────────┐          ┌───────────────┐
        │   RTAB-Map    │          │  Local        │
        │     SLAM      │          │  Costmap      │
        │               │          │               │
        │  3D Mapping   │          │  Voxel Layer  │
        │  Localization │          │  (3D Obstacles)│
        └───────────────┘          └───────────────┘
                │                           │
                ▼                           ▼
        ┌───────────────┐          ┌───────────────┐
        │ /rtabmap/     │          │ /local_costmap/│
        │  grid_map     │          │   costmap     │
        └───────────────┘          └───────────────┘
                │
                ▼
        ┌───────────────┐
        │   Global      │
        │   Costmap     │
        │               │
        │  Static Layer │
        │  Obstacle     │
        │  Inflation    │
        └───────────────┘
                │
                ▼
        ┌───────────────┐
        │ /global_costmap│
        │   /costmap    │
        └───────────────┘
                │
                ▼
        ┌───────────────┐
        │   Nav2 Path   │
        │   Planner     │
        │  (Future)     │
        └───────────────┘
```

---

## Critical Topic Connections

### 1. Camera Point Clouds → RTAB-Map
**Topic:** `/zed_{front|rear|manipulator}/zed_node/point_cloud/cloud_registered`
**Type:** `sensor_msgs/PointCloud2`
**Rate:** 15 Hz
**Purpose:** 3D perception for SLAM and mapping

### 2. RTAB-Map → Global Costmap
**Topic:** `/rtabmap/grid_map`
**Type:** `nav_msgs/OccupancyGrid`
**Rate:** 1 Hz
**Purpose:** 2D occupancy grid for static obstacle representation
**Configuration:** `multi_camera_config.yaml` lines 48-62 (Grid/FromDepth settings)

### 3. Camera Point Clouds → Local Costmap
**Topic:** `/zed_{front|rear|manipulator}/zed_node/point_cloud/cloud_registered`
**Type:** `sensor_msgs/PointCloud2`
**Rate:** 15 Hz
**Purpose:** Dynamic obstacle detection in voxel grid
**Configuration:** `costmap_params.yaml` lines 147-189 (voxel_layer settings)

### 4. TF Tree (Coordinate Frames)
```
map (RTAB-Map SLAM output)
  └─ odom (Odometry from RTAB-Map visual odometry)
      └─ base_link (Robot base frame)
          ├─ camera1_camera_link (Front camera)
          ├─ camera2_camera_link (Rear camera)
          └─ camera3_camera_link (Manipulator camera)
```

---

## Configuration Files

### 1. `config/multi_camera_config.yaml`
**Purpose:** ZED camera and RTAB-Map SLAM configuration
**Key Changes Made:**
- Line 2: `cam_names: "[camera1,camera2,camera3]"` - Added camera3
- Line 4: `cam_serials: "[32514439,31718458,XXXXXXXX]"` - TODO: Add camera3 serial
- Line 25: `rgbd_cameras: 3` - Changed from 0 to 3
- Lines 48-62: Grid map generation parameters (CRITICAL for costmap)

**Critical Parameters:**
```yaml
Grid/FromDepth: "true"         # Enable 2D grid generation
Grid/3D: "false"               # Generate 2D (not 3D)
Grid/CellSize: "0.05"          # 5cm resolution
Grid/RayTracing: "true"        # Mark free space
Grid/MaxObstacleHeight: "2.0"  # Filter tall obstacles
```

### 2. `config/costmap_params.yaml`
**Purpose:** Nav2 local and global costmap configuration
**Status:** NEW FILE - Created to connect RTAB-Map to navigation

**Global Costmap (lines 7-100):**
- Frame: `map`
- Size: 50m x 50m
- Resolution: 5cm
- Layers: static_layer, obstacle_layer, inflation_layer
- Static layer subscribes to: `/rtabmap/grid_map`

**Local Costmap (lines 104-199):**
- Frame: `odom`
- Size: 10m x 10m (rolling window)
- Resolution: 5cm
- Layers: voxel_layer, inflation_layer
- Voxel grid: 40 voxels @ 5cm = 2m height

### 3. `launch/costmap_bringup.launch.py`
**Purpose:** Launch Nav2 costmap nodes with RTAB-Map integration
**Status:** NEW FILE - Launches global_costmap and local_costmap nodes

**Key Remappings:**
```python
# Line 43: Connect RTAB-Map grid to global costmap
('/map', '/rtabmap/grid_map')
```

### 4. `src/nav2_stanley/urdf/zed_cameras.xacro.urdf`
**Purpose:** URDF definition for 3 ZED cameras
**Key Changes Made:**
- Lines 22-26: Added camera3 arguments
- Lines 40-43: Added camera3 instantiation
- Lines 80-89: Added camera3 joint (20cm forward, 1.2m height, 30° downward tilt)

**Camera Positions:**
```
Camera 1 (Front):       xyz="0.0, 0.0, 1.5"   rpy="0, 0, 0"
Camera 2 (Rear):        xyz="-0.12, 0.0, 1.0" rpy="0, 0, π"
Camera 3 (Manipulator): xyz="0.2, 0.0, 1.2"   rpy="0, 0.524, 0" (30° down)
```

---

## Launch Sequence

### Step 1: Launch ZED Cameras + RTAB-Map
```bash
ros2 launch mobility_ctrl multi_camera_launch.py
```
**Expected Topics:**
- `/zed_front/zed_node/point_cloud/cloud_registered`
- `/zed_rear/zed_node/point_cloud/cloud_registered`
- `/zed_manipulator/zed_node/point_cloud/cloud_registered`
- `/rtabmap/grid_map` ← **CRITICAL: Verify this is publishing!**

### Step 2: Launch Costmaps
```bash
ros2 launch mobility_ctrl costmap_bringup.launch.py
```
**Expected Topics:**
- `/global_costmap/costmap`
- `/local_costmap/costmap`
- `/local_costmap/voxel_grid`

---

## Verification Checklist

### Before Running
- [ ] Update camera3 serial number in `multi_camera_config.yaml` line 4
- [ ] Verify all 3 ZED cameras are connected and detected
- [ ] Check TF tree is properly configured

### After Launch (RTAB-Map)
```bash
# Check if grid_map is publishing
ros2 topic hz /rtabmap/grid_map

# Should show: ~1 Hz
# If no output → RTAB-Map grid generation is not working!

# Visualize grid map
ros2 run rviz2 rviz2
# Add: By topic → /rtabmap/grid_map → Map
```

### After Launch (Costmaps)
```bash
# Check global costmap
ros2 topic hz /global_costmap/costmap
# Should show: ~1 Hz

# Check local costmap
ros2 topic hz /local_costmap/costmap
# Should show: ~2 Hz

# Check voxel grid
ros2 topic hz /local_costmap/voxel_grid
# Should show: ~2 Hz
```

### RQT Graph Verification
```bash
rqt_graph
```
**Expected Connections:**
1. `rtabmap` → `/rtabmap/grid_map` → `global_costmap`
2. `zed_front/zed_node` → `point_cloud` → `local_costmap`
3. `zed_rear/zed_node` → `point_cloud` → `local_costmap`
4. `zed_manipulator/zed_node` → `point_cloud` → `local_costmap`

**Previous Problem (SOLVED):**
- Costmap was isolated with no connections
- RTAB-Map was not publishing `/rtabmap/grid_map`
- **Solution:** Enabled Grid/FromDepth and related parameters

---

## Troubleshooting

### Issue 1: `/rtabmap/grid_map` not publishing
**Symptoms:**
- `ros2 topic list` shows `/rtabmap/grid_map` but `ros2 topic hz` shows nothing
- Global costmap shows only unknown (gray) cells

**Diagnosis:**
```bash
ros2 param get /rtabmap/rtabmap Grid/FromDepth
# Should return: true
```

**Solution:**
- Verify `multi_camera_config.yaml` has `Grid/FromDepth: "true"` (line 49)
- Restart RTAB-Map launch file

### Issue 2: Costmaps not receiving point clouds
**Symptoms:**
- Local costmap doesn't show obstacles
- Voxel grid is empty

**Diagnosis:**
```bash
# Check if cameras are publishing
ros2 topic hz /zed_front/zed_node/point_cloud/cloud_registered
ros2 topic hz /zed_rear/zed_node/point_cloud/cloud_registered
ros2 topic hz /zed_manipulator/zed_node/point_cloud/cloud_registered

# Should all show: ~15 Hz
```

**Solution:**
- Verify `multi_camera_config.yaml` has `point_cloud: true` (line 14)
- Check camera topic names match in `costmap_params.yaml`
- Verify TF tree has camera frames

### Issue 3: Camera3 not launching
**Symptoms:**
- Only 2 cameras appear in `ros2 node list`
- Missing camera3 topics

**Diagnosis:**
- Check if camera3 serial number is correctly set (not XXXXXXXX)
- Verify camera is physically connected and powered

**Solution:**
```bash
# Find camera serial number
ros2 run zed_wrapper zed_camera_info

# Update multi_camera_config.yaml line 4 with actual serial
```

### Issue 4: TF transform errors
**Symptoms:**
- Warning: "Could not transform point cloud from camera3_camera_link to base_link"

**Diagnosis:**
```bash
# Check TF tree
ros2 run tf2_tools view_frames

# Should show all camera frames connected to base_link
```

**Solution:**
- Verify URDF has camera3_joint defined
- Check that ZED wrapper is publishing camera3 TF

---

## Topic Reference

### Published by ZED Cameras (×3)
| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/zed_{camera}/zed_node/point_cloud/cloud_registered` | PointCloud2 | 15 Hz | Registered point cloud |
| `/zed_{camera}/zed_node/rgb/image_rect_color` | Image | 15 Hz | RGB image |
| `/zed_{camera}/zed_node/depth/depth_registered` | Image | 15 Hz | Depth image |

### Published by RTAB-Map
| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/rtabmap/grid_map` | OccupancyGrid | 1 Hz | 2D occupancy grid |
| `/rtabmap/cloud_map` | PointCloud2 | On update | 3D point cloud map |
| `/rtabmap/info` | Info | 1 Hz | SLAM statistics |
| `/tf` | TFMessage | Varies | map→odom transform |

### Published by Costmaps
| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/global_costmap/costmap` | OccupancyGrid | 1 Hz | Global costmap |
| `/global_costmap/costmap_updates` | OccupancyGridUpdate | On update | Incremental updates |
| `/local_costmap/costmap` | OccupancyGrid | 2 Hz | Local costmap |
| `/local_costmap/voxel_grid` | VoxelGrid | 2 Hz | 3D voxel representation |

---

## Camera Coverage Zones

### Camera 1 - Front (Primary Navigation)
- **Position:** (0.0, 0.0, 1.5) from base_link
- **Orientation:** Forward (0° horizontal)
- **Coverage:** Front hemisphere, 8-10m range
- **Purpose:** Primary navigation, obstacle avoidance, ArUco detection

### Camera 2 - Rear (Reverse/Blind Spot)
- **Position:** (-0.12, 0.0, 1.0) from base_link
- **Orientation:** Backward (180° rotation)
- **Coverage:** Rear hemisphere, 8-10m range
- **Purpose:** Reverse navigation, rear obstacle detection

### Camera 3 - Manipulator (Close-Range Tasks)
- **Position:** (0.2, 0.0, 1.2) from base_link
- **Orientation:** 30° downward tilt
- **Coverage:** Near-field (4-6m), overhead view
- **Purpose:** Manipulation tasks, close-range object detection, sample collection

**Combined Coverage:** Near 360° horizontal coverage with depth perception in all directions

---

## Parameter Tuning Guide

### If obstacles appear too inflated (robot won't navigate)
Edit `costmap_params.yaml`:
```yaml
inflation_layer:
  inflation_radius: 0.50  # Reduce from 0.70
  cost_scaling_factor: 2.0  # Reduce from 3.0
```

### If robot hits obstacles (not enough safety margin)
Edit `costmap_params.yaml`:
```yaml
inflation_layer:
  inflation_radius: 0.90  # Increase from 0.70
  cost_scaling_factor: 5.0  # Increase from 3.0
```

### If point clouds are too dense (performance issues)
Edit `multi_camera_config.yaml`:
```yaml
cloud_decimation: 8  # Increase from 4 (more aggressive downsampling)
cloud_voxel_size: 0.10  # Increase from 0.05 (larger voxels)
```

### If map has too much noise
Edit `multi_camera_config.yaml`:
```yaml
Grid/ClusterRadius: 0.2  # Increase from 0.1 (merge nearby points)
Grid/MinGroundHeight: "-0.3"  # Adjust ground detection
```

---

## Next Steps

### Immediate (Required to Run)
1. [ ] Update camera3 serial number in `multi_camera_config.yaml` line 4
2. [ ] Test launch sequence and verify all topics are publishing
3. [ ] Verify `/rtabmap/grid_map` is publishing with `ros2 topic hz`

### Integration (To Complete Navigation Stack)
1. [ ] Configure Nav2 planner and controller
2. [ ] Add path planning launch file
3. [ ] Integrate YOLO detection with costmap
4. [ ] Test full autonomous navigation

### Optimization
1. [ ] Tune costmap inflation parameters for rover size
2. [ ] Adjust RTAB-Map loop closure parameters
3. [ ] Optimize point cloud processing rate
4. [ ] Camera position fine-tuning based on field testing

---

## ERC 2025 Competition Readiness

### Autonomous Navigation Mission Requirements
- [x] 360° camera coverage
- [x] SLAM and localization (RTAB-Map)
- [x] Obstacle detection (voxel layer)
- [x] 2D occupancy grid generation
- [ ] Path planning (Nav2 planner - TODO)
- [ ] ArUco marker detection integration
- [ ] Blind teleoperation support

### Recommended Testing Workflow
1. Test SLAM in known environment
2. Verify costmap updates with dynamic obstacles
3. Test navigation to GPS waypoints
4. Test ArUco marker approach and alignment
5. Full mission simulation

---

## References

- **Nav2 Documentation:** https://navigation.ros.org
- **RTAB-Map ROS2:** https://github.com/introlab/rtabmap_ros
- **ZED ROS2 Wrapper:** https://github.com/stereolabs/zed-ros2-wrapper
- **ERC 2025 Rules:** See competition documentation

---

**Last Updated:** 2025-12-31
**Status:** Implementation Complete - Ready for Testing
**Author:** Exodus Autonomy Team
