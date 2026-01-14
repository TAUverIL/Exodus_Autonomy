# Team Task Implementation Guides

**Created:** 2026-01-13
**Project:** Exodus Autonomy - 3 ZED2i Camera System

---

## Task Overview

| Task | Owner | Difficulty | Est. Lines of Code |
|------|-------|------------|-------------------|
| 1. 3D Position Calculation | Person 1 | Medium | ~50 lines |
| 2. Continuous Replanning | Person 5 | Medium | ~40 lines |
| 3. Wheel Odometry Integration | Person 4 | Easy | ~30 lines |
| 4. Camera-to-Base Transform | Person 2 | Hard | ~60 lines |
| 5. RTAB-Map Tuning | Person 3 | Medium | Config changes |

---

# Task 1: 3D Position Calculation from Camera Intrinsics

**Owner:** Person 1 (Perception - Detection)
**File:** `src/autonomy_node/autonomy_node/detector_3cam.py`

## Problem

Currently lines 154-157 hardcode position:
```python
obs.position_3d = Point()
obs.position_3d.x = 0.0  # TODO: Calculate from camera intrinsics
obs.position_3d.y = 0.0
obs.position_3d.z = median_distance
```

## Goal

Calculate real X, Y, Z coordinates using the pinhole camera model.

## Background: Pinhole Camera Model

```
Given:
- (u, v) = pixel coordinates of detection center
- Z = depth at that pixel (from depth image)
- fx, fy = focal lengths (from camera_info)
- cx, cy = principal point (from camera_info)

Calculate:
- X = (u - cx) * Z / fx
- Y = (v - cy) * Z / fy
```

## Implementation Steps

### Step 1: Add CameraInfo subscription

Add these imports at the top:
```python
from sensor_msgs.msg import Image, CameraInfo
```

Add camera info storage in `__init__`:
```python
# Camera intrinsics storage
self.camera_info = {}

# Subscribe to camera info for each camera
self.sub_cam1_info = self.create_subscription(
    CameraInfo,
    '/zed_multi/camera1/rgb/color/rect/camera_info',
    lambda msg: self.camera_info_callback(msg, 'camera1'),
    10
)
self.sub_cam2_info = self.create_subscription(
    CameraInfo,
    '/zed_multi/camera2/rgb/color/rect/camera_info',
    lambda msg: self.camera_info_callback(msg, 'camera2'),
    10
)
self.sub_cam3_info = self.create_subscription(
    CameraInfo,
    '/zed_multi/camera3/rgb/color/rect/camera_info',
    lambda msg: self.camera_info_callback(msg, 'camera3'),
    10
)
```

### Step 2: Add camera info callback

```python
def camera_info_callback(self, msg, camera_name):
    """Store camera intrinsics"""
    # K matrix: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
    self.camera_info[camera_name] = {
        'fx': msg.k[0],
        'fy': msg.k[4],
        'cx': msg.k[2],
        'cy': msg.k[5],
        'width': msg.width,
        'height': msg.height
    }
    self.get_logger().info(f'{camera_name} intrinsics: fx={msg.k[0]:.1f}, fy={msg.k[4]:.1f}', once=True)
```

### Step 3: Update process_camera to pass camera name

Change the lambda callbacks to include camera name:
```python
self.ts_cam1.registerCallback(
    lambda rgb, depth: self.process_camera(rgb, depth, "camera1_left_camera_frame", "Camera 1 (Front)", "camera1")
)
```

Update function signature:
```python
def process_camera(self, rgb_msg, depth_msg, frame_id, window_name, camera_name):
```

### Step 4: Calculate 3D position in process_detections

Update `process_detections` signature:
```python
def process_detections(self, boxes, annotated_frame, depth_image, detection_msg, camera_name):
```

Replace the position calculation (lines 148-157):
```python
# Calculate bounding box center
center_u = (x1 + x2) / 2.0
center_v = (y1 + y2) / 2.0

# Get camera intrinsics
if camera_name in self.camera_info:
    info = self.camera_info[camera_name]
    fx = info['fx']
    fy = info['fy']
    cx = info['cx']
    cy = info['cy']

    # Calculate 3D position using pinhole model
    Z = median_distance  # depth
    X = (center_u - cx) * Z / fx
    Y = (center_v - cy) * Z / fy
else:
    # Fallback if no camera info yet
    X = 0.0
    Y = 0.0
    Z = median_distance

# Create obstacle message
obs = Obstacle()
obs.class_label = label_name
obs.confidence = conf
obs.distance_2d = median_distance

obs.position_3d = Point()
obs.position_3d.x = float(X)  # Forward (camera Z)
obs.position_3d.y = float(Y)  # Left (camera X)
obs.position_3d.z = float(Z)  # Depth
```

**Note:** ZED uses camera convention where Z is forward. You may need to swap axes depending on your frame convention.

## Testing

```bash
# Build
colcon build --packages-select autonomy_node --symlink-install

# Run detector
ros2 run autonomy_node detector_3cam

# Check output
ros2 topic echo /autonomy/obstacle_detections

# Verify position_3d has real values (not 0.0)
```

## Done When

- `/autonomy/obstacle_detections` messages have real `position_3d.x`, `position_3d.y`, `position_3d.z` values
- Values change based on where object is in camera frame
- Objects to the left have negative Y, objects to the right have positive Y

---

# Task 2: Continuous Path Replanning

**Owner:** Person 5 (Navigation)
**File:** `src/autonomy_node/autonomy_node/path_planner.py`

## Problem

Current path planner only plans once when a goal is received (line 81):
```python
def goal_callback(self, msg):
    self.goal_pose = msg.pose
    self.plan_path()  # Only plans once!
```

## Goal

Implement continuous replanning that triggers when:
1. Timer fires (every 2-3 seconds)
2. Robot pose drifts too far from planned path
3. New obstacles block current path

## Implementation Steps

### Step 1: Add replanning parameters

In `__init__`, add:
```python
# Replanning parameters
self.declare_parameter('replan_interval', 3.0)  # seconds
self.declare_parameter('path_deviation_threshold', 0.5)  # meters
self.declare_parameter('obstacle_check_distance', 2.0)  # meters ahead

self.replan_interval = self.get_parameter('replan_interval').value
self.path_deviation_threshold = self.get_parameter('path_deviation_threshold').value
self.obstacle_check_distance = self.get_parameter('obstacle_check_distance').value

# Store current planned path
self.current_path = None
self.path_index = 0  # Current waypoint robot is heading to

# Create replan timer
self.replan_timer = self.create_timer(self.replan_interval, self.check_replan)
```

### Step 2: Store path after planning

At end of `plan_path()`, add:
```python
# Store path for replanning checks
self.current_path = simplified_path
self.path_index = 0

self.get_logger().info('Path published!')
```

### Step 3: Add replan checking function

```python
def check_replan(self):
    """Periodically check if replanning is needed"""
    if self.goal_pose is None or self.current_path is None:
        return

    if self.current_pose is None or self.occupancy_grid is None:
        return

    needs_replan = False
    reason = ""

    # Check 1: Path deviation
    if self.check_path_deviation():
        needs_replan = True
        reason = "Robot deviated from path"

    # Check 2: Obstacle on path
    elif self.check_obstacle_on_path():
        needs_replan = True
        reason = "Obstacle detected on path"

    # Check 3: Goal changed significantly (handled by goal_callback)

    if needs_replan:
        self.get_logger().info(f'Replanning: {reason}')
        self.plan_path()
```

### Step 4: Implement path deviation check

```python
def check_path_deviation(self):
    """Check if robot has deviated too far from planned path"""
    if self.current_path is None or len(self.current_path) < 2:
        return False

    robot_x = self.current_pose.position.x
    robot_y = self.current_pose.position.y

    # Find closest point on path
    min_dist = float('inf')
    for i, (grid_x, grid_y) in enumerate(self.current_path):
        world_x, world_y = self.grid_to_world(grid_x, grid_y, self.occupancy_grid)
        dist = math.sqrt((robot_x - world_x)**2 + (robot_y - world_y)**2)
        if dist < min_dist:
            min_dist = dist
            self.path_index = i

    return min_dist > self.path_deviation_threshold
```

### Step 5: Implement obstacle-on-path check

```python
def check_obstacle_on_path(self):
    """Check if any obstacles are blocking the path ahead"""
    if self.current_path is None or self.occupancy_grid is None:
        return False

    robot_x = self.current_pose.position.x
    robot_y = self.current_pose.position.y

    # Check path points ahead of robot
    for i in range(self.path_index, len(self.current_path)):
        grid_x, grid_y = self.current_path[i]
        world_x, world_y = self.grid_to_world(grid_x, grid_y, self.occupancy_grid)

        # Only check within obstacle_check_distance
        dist_to_point = math.sqrt((robot_x - world_x)**2 + (robot_y - world_y)**2)
        if dist_to_point > self.obstacle_check_distance:
            break

        # Check if this cell is now occupied
        if not self.is_valid_cell(grid_x, grid_y, self.occupancy_grid):
            self.get_logger().warn(f'Obstacle at ({world_x:.1f}, {world_y:.1f}) blocks path')
            return True

    return False
```

### Step 6: Update goal callback to reset path

```python
def goal_callback(self, msg):
    """Received new goal - plan path"""
    self.goal_pose = msg.pose
    self.current_path = None  # Reset current path
    self.path_index = 0
    self.get_logger().info(f'New goal received: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')
    self.plan_path()
```

## Testing

```bash
# Terminal 1: Run path planner
ros2 run autonomy_node path_planner

# Terminal 2: Send a goal
ros2 topic pub /goal_pose geometry_msgs/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {position: {x: 5.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}
}"

# Terminal 3: Watch for replanning logs
# Move robot or add obstacles - planner should replan automatically
```

## Done When

- Path planner logs "Replanning: ..." every few seconds when conditions change
- New path publishes to `/autonomy/planned_path` when obstacles appear
- Robot doesn't follow stale paths

---

# Task 3: Wheel Odometry Integration

**Owner:** Person 4 (Localization)
**File:** `src/autonomy_node/autonomy_node/odom_fusion.py`

## Problem

Current code subscribes to `/wheel_odom` (line 45-49) but:
1. Doesn't handle case when wheel odom is unavailable
2. Topic name might differ from computer team's actual topic
3. Noise parameters may need tuning

## Goal

- Confirm wheel odometry topic with computer team
- Add graceful handling when wheel odom is missing
- Make the system work with visual-only when wheel odom unavailable

## Implementation Steps

### Step 1: Make wheel odometry topic configurable

In `__init__`, add parameter:
```python
# Configurable wheel odometry topic
self.declare_parameter('wheel_odom_topic', '/wheel_odom')
self.declare_parameter('visual_odom_topic', '/rtabmap/odom')
self.declare_parameter('use_wheel_odom', True)

wheel_odom_topic = self.get_parameter('wheel_odom_topic').value
visual_odom_topic = self.get_parameter('visual_odom_topic').value
self.use_wheel_odom = self.get_parameter('use_wheel_odom').value

# Track data reception
self.last_visual_time = None
self.last_wheel_time = None
self.wheel_odom_received = False
```

### Step 2: Add timeout handling

Add at end of `__init__`:
```python
# Watchdog timer to detect missing odometry sources
self.watchdog_timer = self.create_timer(5.0, self.check_odom_sources)
```

Add watchdog function:
```python
def check_odom_sources(self):
    """Check which odometry sources are active"""
    current_time = self.get_clock().now()

    visual_ok = False
    wheel_ok = False

    if self.last_visual_time is not None:
        visual_age = (current_time - self.last_visual_time).nanoseconds / 1e9
        visual_ok = visual_age < 2.0

    if self.last_wheel_time is not None:
        wheel_age = (current_time - self.last_wheel_time).nanoseconds / 1e9
        wheel_ok = wheel_age < 2.0

    if not visual_ok and not wheel_ok:
        self.get_logger().warn('No odometry data received!')
    elif not wheel_ok and self.use_wheel_odom:
        if not self.wheel_odom_received:
            self.get_logger().info('Wheel odometry not available - using visual only', throttle_duration_sec=10.0)
    elif visual_ok and wheel_ok:
        if not self.wheel_odom_received:
            self.wheel_odom_received = True
            self.get_logger().info('Wheel odometry now available - fusion active')
```

### Step 3: Update callbacks to track timing

Update `zed_odom_callback`:
```python
def zed_odom_callback(self, msg):
    """Process ZED visual odometry"""
    current_time = self.get_clock().now()
    self.last_visual_time = current_time  # Track reception time

    # ... rest of existing code
```

Update `wheel_odom_callback`:
```python
def wheel_odom_callback(self, msg):
    """Process wheel odometry"""
    current_time = self.get_clock().now()
    self.last_wheel_time = current_time  # Track reception time

    # ... rest of existing code
```

### Step 4: Add configurable noise parameters

In `__init__`, make noise configurable:
```python
# Configurable noise parameters
self.declare_parameter('visual_noise', 0.05)
self.declare_parameter('wheel_noise', 0.1)

visual_noise = self.get_parameter('visual_noise').value
wheel_noise = self.get_parameter('wheel_noise').value

# Measurement noise covariance for visual odometry
self.R_visual = np.eye(6) * visual_noise

# Measurement noise covariance for wheel odometry
self.R_wheel = np.eye(6) * wheel_noise
```

### Step 5: Coordinate with computer team

Create a simple test script to verify wheel odometry:
```python
#!/usr/bin/env python3
"""Test script to check wheel odometry topic"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class WheelOdomChecker(Node):
    def __init__(self):
        super().__init__('wheel_odom_checker')

        # Try common topic names
        topics_to_try = ['/wheel_odom', '/odom', '/wheel_odometry', '/encoder_odom']

        for topic in topics_to_try:
            self.create_subscription(Odometry, topic,
                lambda msg, t=topic: self.odom_callback(msg, t), 10)

        self.get_logger().info(f'Checking topics: {topics_to_try}')

    def odom_callback(self, msg, topic):
        self.get_logger().info(f'Received wheel odom on {topic}: x={msg.pose.pose.position.x:.2f}')

def main():
    rclpy.init()
    node = WheelOdomChecker()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

## Questions for Computer Team

1. What is the exact topic name for wheel odometry?
2. What frame_id does it use? (`odom`? `base_link`?)
3. What is the publish rate? (10Hz? 50Hz?)
4. Does it include covariance?

## Testing

```bash
# Check what odometry topics exist
ros2 topic list | grep odom

# Run fusion with debug
ros2 run autonomy_node odom_fusion --ros-args -p wheel_odom_topic:=/wheel_odom

# Monitor fused output
ros2 topic hz /autonomy/fused_odom
ros2 topic echo /autonomy/fused_odom
```

## Done When

- Node starts without errors even when wheel odom unavailable
- Logs clearly indicate when wheel odom connects/disconnects
- `/autonomy/fused_odom` publishes at ~30Hz using both sources

---

# Task 4: Camera-to-Base Transform for Obstacles

**Owner:** Person 2 (Perception - Grid Mapping)
**Files:**
- `src/autonomy_node/autonomy_node/detector_3cam.py`
- `src/autonomy_node/autonomy_node/grid_mapper_3cam.py`

## Problem

Current `grid_mapper_3cam.py` uses hardcoded camera positions (lines 89-99):
```python
if frame_id == "camera1":  # Front camera
    x = distance  # Forward
    y = 0.0
elif frame_id == "camera2":  # Left camera (example)
    x = 0.0
    y = distance
```

This doesn't account for:
1. Actual camera mounting positions on the robot
2. Camera orientations
3. Real 3D position from Task 1

## Goal

Use TF2 to properly transform obstacle positions from camera frames to `base_link`.

## Implementation Steps

### Step 1: Add TF2 to detector_3cam.py

Add imports:
```python
import tf2_ros
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped
```

Add TF buffer in `__init__`:
```python
# TF2 for coordinate transforms
self.tf_buffer = tf2_ros.Buffer()
self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
```

### Step 2: Add transform function to detector

```python
def transform_point_to_base(self, x, y, z, source_frame, stamp):
    """Transform a point from camera frame to base_link"""
    try:
        # Create point in source frame
        point_stamped = PointStamped()
        point_stamped.header.frame_id = source_frame
        point_stamped.header.stamp = stamp
        point_stamped.point.x = float(z)  # Camera Z -> forward
        point_stamped.point.y = float(-x)  # Camera X -> left (negated)
        point_stamped.point.z = float(-y)  # Camera Y -> up (negated)

        # Transform to base_link
        transform = self.tf_buffer.lookup_transform(
            'base_link',
            source_frame,
            stamp,
            timeout=rclpy.duration.Duration(seconds=0.1)
        )

        transformed = do_transform_point(point_stamped, transform)
        return transformed.point.x, transformed.point.y, transformed.point.z

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException) as e:
        self.get_logger().warn(f'TF transform failed: {e}', throttle_duration_sec=5.0)
        return None, None, None
```

### Step 3: Update process_detections to use transforms

After calculating X, Y, Z from camera intrinsics, add:
```python
# Transform to base_link frame
base_x, base_y, base_z = self.transform_point_to_base(
    X, Y, Z,
    detection_msg.header.frame_id,
    detection_msg.header.stamp
)

if base_x is not None:
    obs.position_3d.x = base_x
    obs.position_3d.y = base_y
    obs.position_3d.z = base_z
else:
    # Fallback to camera frame position
    obs.position_3d.x = float(Z)  # Forward
    obs.position_3d.y = float(-X)  # Left
    obs.position_3d.z = float(-Y)  # Up
```

### Step 4: Update grid_mapper to use transformed positions

Replace the hardcoded mapping in `detection_callback`:
```python
def detection_callback(self, msg):
    """Process obstacle detections and update grid"""
    for detection in msg.detections:
        # Use transformed 3D position (now in base_link frame)
        x = detection.position_3d.x  # Forward in base_link
        y = detection.position_3d.y  # Left in base_link

        # Validate position
        if abs(x) < 0.1 and abs(y) < 0.1:
            # Position still at origin - likely no transform yet
            # Fall back to distance-based estimate
            distance = detection.distance_2d
            frame_id = msg.header.frame_id

            if "camera1" in frame_id:  # Front
                x = distance
                y = 0.0
            elif "camera2" in frame_id:  # Rear
                x = -distance
                y = 0.0
            elif "camera3" in frame_id:  # Manipulator (front-down)
                x = distance * 0.7  # Angled down
                y = 0.0

        # Mark obstacle in grid
        self.mark_obstacle(x, y, detection.confidence)
```

### Step 5: Define camera transforms (if not in URDF)

If camera transforms are not published, you need to publish static transforms.
Create `src/autonomy_node/launch/camera_transforms.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Camera 1 (Front) - mounted 0.3m forward, 0.5m up
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.3', '0', '0.5', '0', '0', '0', 'base_link', 'camera1_left_camera_frame']
        ),
        # Camera 2 (Rear) - mounted 0.3m backward, 0.5m up, rotated 180 degrees
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['-0.3', '0', '0.5', '0', '0', '3.14159', 'base_link', 'camera2_left_camera_frame']
        ),
        # Camera 3 (Manipulator) - mounted 0.2m forward, 0.3m up, tilted down 30 degrees
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.2', '0', '0.3', '0', '0.523', '0', 'base_link', 'camera3_left_camera_frame']
        ),
    ])
```

## Testing

```bash
# Check TF tree
ros2 run tf2_tools view_frames

# Verify transforms exist
ros2 run tf2_ros tf2_echo base_link camera1_left_camera_frame

# Run detector and check positions
ros2 run autonomy_node detector_3cam
ros2 topic echo /autonomy/obstacle_detections

# Visualize in RViz
# Add TF display and PointStamped for obstacle positions
```

## Done When

- Obstacles from camera 1 (front) appear in front of robot on grid
- Obstacles from camera 2 (rear) appear behind robot on grid
- Obstacles from camera 3 (manipulator) appear at correct position
- Grid map shows obstacles in correct world positions

---

# Task 5: RTAB-Map Parameter Tuning

**Owner:** Person 3 (SLAM)
**File:** `src/multi_zed_rtab/config/multi_camera_config.yaml`

## Problem

Current configuration may not be optimal for:
1. Real-time performance on Jetson Orin NX
2. 3-camera setup resource usage
3. Map quality vs. CPU trade-off

## Goal

Test and tune RTAB-Map parameters for stable 1Hz grid_map output with <80% CPU usage.

## Current Configuration Review

Key parameters in `multi_camera_config.yaml`:
```yaml
Rtabmap/DetectionRate: "1.0"      # 1 Hz - controls processing rate
Vis/MaxFeatures: "400"            # Features per image
Kp/MaxFeatures: "400"             # Loop closure features
cloud_decimation: 4               # Point cloud reduction
cloud_voxel_size: 0.05            # 5cm voxel filter
```

## Tuning Steps

### Step 1: Baseline measurement

```bash
# Terminal 1: Start RTAB-Map
ros2 launch multi_zed_rtab multi_zed_rtab.launch.py

# Terminal 2: Monitor CPU/GPU
htop  # or tegrastats on Jetson

# Terminal 3: Check output rate
ros2 topic hz /rtabmap/grid_map
ros2 topic hz /rtabmap/odom

# Record baseline values:
# - CPU usage: ____%
# - grid_map rate: ____ Hz
# - odom rate: ____ Hz
```

### Step 2: Performance tuning options

**If CPU > 80%**, try these changes one at a time:

#### Option A: Reduce feature count
```yaml
Vis/MaxFeatures: "300"    # Reduce from 400
Kp/MaxFeatures: "300"     # Reduce from 400
```

#### Option B: Increase point cloud decimation
```yaml
cloud_decimation: 8       # Increase from 4 (use every 8th point)
cloud_voxel_size: 0.1     # Increase from 0.05 (10cm voxels)
```

#### Option C: Reduce detection rate
```yaml
Rtabmap/DetectionRate: "0.5"  # Process every 2 seconds instead of 1
```

#### Option D: Disable expensive features
```yaml
RGBD/LoopClosureReextractFeatures: "false"  # Don't re-extract features
RGBD/ProximityBySpace: "false"              # Disable proximity detection
```

### Step 3: Quality tuning options

**If map quality is poor**, try these:

#### Option A: Increase features
```yaml
Vis/MaxFeatures: "600"    # More features
Vis/MinInliers: "20"      # Require more inliers
```

#### Option B: Better depth filtering
```yaml
cloud_max_depth: 8.0      # Reduce from 10m (less noise)
cloud_min_depth: 0.5      # Increase from 0.3m (avoid near noise)
```

#### Option C: Grid map quality
```yaml
GridGlobal/OccupancyThr: "0.6"   # Higher threshold (more certain obstacles)
GridGlobal/ProbHit: "0.8"        # Higher hit probability
GridGlobal/ProbMiss: "0.3"       # Lower miss probability
```

### Step 4: Create tuning presets

Create different config files for testing:

**config/rtabmap_performance.yaml** (for slow hardware):
```yaml
Vis/MaxFeatures: "200"
Kp/MaxFeatures: "200"
cloud_decimation: 8
Rtabmap/DetectionRate: "0.5"
```

**config/rtabmap_quality.yaml** (for good hardware):
```yaml
Vis/MaxFeatures: "600"
Kp/MaxFeatures: "600"
cloud_decimation: 2
Rtabmap/DetectionRate: "2.0"
```

### Step 5: Document findings

Create a tuning report with:

| Parameter | Default | Tested | Result |
|-----------|---------|--------|--------|
| Vis/MaxFeatures | 400 | 300 | CPU -10%, quality OK |
| cloud_decimation | 4 | 8 | CPU -15%, slight quality loss |
| DetectionRate | 1.0 | 0.5 | CPU -20%, slower updates |

## Jetson-Specific Optimization

```bash
# Set Jetson to max performance mode
sudo nvpmodel -m 0
sudo jetson_clocks

# Monitor with tegrastats
tegrastats

# Expected baseline:
# - CPU: ~60-70%
# - GPU: ~30-40%
# - RAM: ~6-8GB
```

## Testing Checklist

- [ ] `/rtabmap/grid_map` publishing at 1Hz (stable)
- [ ] `/rtabmap/odom` publishing at 15-30Hz
- [ ] CPU usage < 80%
- [ ] GPU usage < 60%
- [ ] No memory leaks (RAM stable over 10 minutes)
- [ ] Map quality acceptable in RViz

## Done When

- Documented optimal parameters for Jetson Orin NX
- `/rtabmap/grid_map` stable at ~1Hz
- System runs for 30+ minutes without degradation
- Created final `multi_camera_config.yaml` with tested values

---

# Communication Protocol

## Daily Standups

Each person reports:
1. What I completed yesterday
2. What I'm working on today
3. Any blockers

## Code Review Process

1. Create branch: `feature/task-N-description`
2. Make changes, test locally
3. Push and create PR to `develop`
4. Team lead reviews and merges
5. Test on Jetson after merge

## Slack/Discord Channels

- `#autonomy-general` - Daily updates
- `#autonomy-perception` - Tasks 1, 4
- `#autonomy-navigation` - Tasks 2, 3
- `#autonomy-slam` - Task 5

---

**Questions?** Ask the team lead or post in the appropriate channel.
