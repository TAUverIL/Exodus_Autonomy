# Exodus_Autonomy


Autonomy stack for Team Exodus — designed to run on NVIDIA Jetson Orin NX and tested in Gazebo + RViz.


**Features**
- ROS2 (Humble) based modular architecture
- Perception: multi-stereo depth, marker detection (ARuco)
- Localization: EKF fusion (IMU, odom, optional GNSS)
- Mapping: local & global map modules
- Planning: global and local planners with dynamic replanning
- Obstacle avoidance: multi-camera fused costmap
- Drone interface: mission commands & state exchange
- Runs in simulation (Gazebo) and on actual robot


## Quick start (developer)


### Prerequisites
- Docker (with NVIDIA Container Toolkit for Jetson)
- ROS 2 Humble installed locally for development (optional)
- Python 3.10+ (for helper scripts)


### Clone
```bash
git clone https://github.com/<your-org>/exodus-autonomy.git
cd exodus-autonomy
