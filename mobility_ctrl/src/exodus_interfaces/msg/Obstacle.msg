# exodus_interfaces/msg/Obstacle.msg

# The actual detection data for a single object
string class_label        # e.g., "person", "microwave"
float32 confidence        # YOLO confidence score (0.0 to 1.0)

# 3D position of the object (median point of the bounding box relative to camera)
# Measured in meters (x, y, z)
geometry_msgs/Point position_3d 

# Distance from the camera to the object's center (2D distance in meters)
float32 distance_2d       # Median distance in meters