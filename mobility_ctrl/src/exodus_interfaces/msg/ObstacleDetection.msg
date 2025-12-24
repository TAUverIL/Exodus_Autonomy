# exodus_interfaces/msg/ObstacleDetection.msg

# Header contains timestamp and frame_id (e.g., 'zed_front_camera_link')
std_msgs/Header header

# Array of individual detection results, referencing the new file
Obstacle[] detections