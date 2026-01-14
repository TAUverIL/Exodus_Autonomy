# Exodus Autonomy - Session Log

> **READ THIS FILE AT THE START OF EVERY SESSION**
> This file tracks progress across Claude sessions. Update it at the end of each session.

---

## Quick Status

| Item | Status |
|------|--------|
| **Current Phase** | Phase 1 - Jetson Verification & Team Task Assignment |
| **Last Session** | 2026-01-13 |
| **Blocking Issues** | None |
| **Next Action** | Commit changes, deploy to Jetson, team starts 5 tasks |

---

## Session History

### Session 5 - 2026-01-13
**Focus:** Team task division and implementation guides

**Completed:**
- [x] Created team structure for 6 people (lead + 5 members)
- [x] Defined 5 concrete tasks with clear ownership
- [x] Created `TEAM_TASKS.md` with detailed implementation guides for all 5 tasks
- [x] Created this `SESSION_LOG.md` for progress tracking

**Tasks Assigned:**
| Task | Owner | File | Status |
|------|-------|------|--------|
| 1. 3D Position Calculation | Person 1 | `detector_3cam.py` | Not Started |
| 2. Continuous Replanning | Person 5 | `path_planner.py` | Not Started |
| 3. Wheel Odometry Integration | Person 4 | `odom_fusion.py` | Not Started |
| 4. Camera-to-Base Transform | Person 2 | `detector_3cam.py`, `grid_mapper_3cam.py` | Not Started |
| 5. RTAB-Map Tuning | Person 3 | `multi_camera_config.yaml` | Not Started |

**Uncommitted Changes:**
- `.gitignore` - Updated
- `JETSON_DEPLOYMENT.md` - Added progress tracker
- `detector_3cam.py` - Fixed topic names and frame_ids
- `follow_waypoints.py` - Fixed waypoint topic
- `TEAM_TASKS.md` - NEW: Implementation guides
- `SESSION_LOG.md` - NEW: This file

**Next Session Should:**
1. Commit all pending changes
2. Push to remote for team to pull
3. Verify team has started their tasks
4. Deploy to Jetson and run verification tests

---

### Session 4 - 2026-01-07
**Focus:** Topic fixes and Jetson deployment prep

**Completed:**
- [x] Fixed `detector_3cam.py` topic names: `/camera1/zed_node_1/...` → `/zed_multi/camera1/...`
- [x] Fixed frame_ids: `camera1` → `camera1_left_camera_frame`
- [x] Fixed `follow_waypoints.py` topic: `waypoints` → `/autonomy/waypoints`
- [x] Added Session Progress Tracker to `JETSON_DEPLOYMENT.md`
- [x] Configured RTAB-Map for RGBDImages interface (`subscribe_rgbd: true`, `rgbd_cameras: 0`)

---

### Session 3 - 2026-01-06 (estimated)
**Focus:** RTAB-Map multi-camera configuration

**Completed:**
- [x] Configured 3-camera RTAB-Map setup
- [x] Created `multi_zed_rtab.launch.py`
- [x] Created `real_nav.launch.py` for costmaps
- [x] Enabled RGB image publishing for cameras
- [x] Fixed `use_sim_time` to FALSE for real hardware

---

### Earlier Sessions
- Initial workspace setup
- Package structure created
- Basic nodes implemented (detector, grid_mapper, odom_fusion, path_planner)
- Custom messages defined (Obstacle, ObstacleDetection)
- Nav2 Stanley controller integration

---

## Project Status Overview

### Packages Status

| Package | Status | Notes |
|---------|--------|-------|
| `autonomy_node` | 80% | Needs Tasks 1,2,3,4 completed |
| `multi_zed_rtab` | 90% | Needs Task 5 tuning |
| `follow_waypoints` | 95% | Topic fix done |
| `exodus_interfaces` | 100% | Complete |
| `nav2_stanley` | 90% | Needs testing |

### Core Nodes Status

| Node | File | Status | Blocking |
|------|------|--------|----------|
| `detector_3cam` | `detector_3cam.py` | Working | Needs 3D position (Task 1) |
| `grid_mapper_3cam` | `grid_mapper_3cam.py` | Working | Needs TF transform (Task 4) |
| `odom_fusion` | `odom_fusion.py` | Working | Needs wheel odom (Task 3) |
| `path_planner` | `path_planner.py` | Working | Needs replanning (Task 2) |
| `follow_waypoints` | `follow_waypoints.py` | Working | None |

### Hardware Testing Status

| Component | Tested | Notes |
|-----------|--------|-------|
| Camera 1 (Front) | No | SN: 32514439 |
| Camera 2 (Rear) | No | SN: 31718458 |
| Camera 3 (Manipulator) | No | SN: 33289078 |
| RTAB-Map SLAM | No | Need Jetson test |
| Costmaps | No | Need Jetson test |
| Path Planning | No | Need Jetson test |

---

## Key Decisions Made

1. **Topic naming convention:** `/zed_multi/camera{1,2,3}/...`
2. **Frame IDs:** `camera{1,2,3}_left_camera_frame`
3. **RTAB-Map interface:** RGBDImages (not individual depth/rgb topics)
4. **Team structure:** 6 people - Lead + Perception(2) + SLAM/Loc(2) + Navigation(1)
5. **Odometry fusion:** Kalman filter combining visual + wheel odom

---

## Known Issues

| Issue | Severity | Workaround | Task to Fix |
|-------|----------|------------|-------------|
| `position_3d.x/y` hardcoded to 0.0 | Medium | Uses distance only | Task 1 |
| No continuous replanning | Medium | Manual replan | Task 2 |
| Wheel odom not connected | Low | Visual odom only | Task 3 |
| Camera positions hardcoded | Medium | Works for front camera | Task 4 |
| RTAB-Map not tuned for Jetson | Low | Default params | Task 5 |

---

## File Reference

### Key Files to Know

```
mobility_ctrl/
├── SESSION_LOG.md          # THIS FILE - read first!
├── TEAM_TASKS.md           # Implementation guides for team
├── JETSON_DEPLOYMENT.md    # Deployment instructions
├── CLAUDE.md               # Project overview for Claude
└── src/
    ├── autonomy_node/
    │   └── autonomy_node/
    │       ├── detector_3cam.py      # YOLO + depth
    │       ├── grid_mapper_3cam.py   # Occupancy grid
    │       ├── odom_fusion.py        # Kalman filter
    │       └── path_planner.py       # A* planning
    ├── multi_zed_rtab/
    │   ├── config/
    │   │   └── multi_camera_config.yaml  # RTAB-Map params
    │   └── launch/
    │       ├── multi_zed_rtab.launch.py  # Camera + SLAM
    │       └── real_nav.launch.py        # Costmaps
    └── follow_waypoints/
        └── follow_waypoints/
            └── follow_waypoints.py   # Nav2 waypoint follower
```

### Key Topics

| Topic | Type | Source |
|-------|------|--------|
| `/zed_multi/camera{1,2,3}/rgb/color/rect/image` | Image | ZED cameras |
| `/zed_multi/camera{1,2,3}/depth/depth_registered` | Image | ZED cameras |
| `/autonomy/obstacle_detections` | ObstacleDetection | detector_3cam |
| `/autonomy/occupancy_grid` | OccupancyGrid | grid_mapper_3cam |
| `/rtabmap/grid_map` | OccupancyGrid | RTAB-Map |
| `/rtabmap/odom` | Odometry | RTAB-Map |
| `/autonomy/fused_odom` | Odometry | odom_fusion |
| `/autonomy/planned_path` | Path | path_planner |
| `/autonomy/waypoints` | Path | path_planner |

---

## Next Session Checklist

When starting a new session:

1. [ ] Read this file first
2. [ ] Check git status for uncommitted changes
3. [ ] Ask user what they want to focus on
4. [ ] Check if team tasks have progressed
5. [ ] Update this file at end of session

---

## How to Update This File

At the end of each session, update:

1. **Quick Status** table at the top
2. Add new **Session History** entry
3. Update **Project Status Overview** if packages changed
4. Update **Known Issues** if fixed or new ones found
5. Update **Next Session Checklist** with specific items

---

**Last Updated:** 2026-01-13
