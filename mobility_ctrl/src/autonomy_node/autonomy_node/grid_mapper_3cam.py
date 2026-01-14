#!/usr/bin/env python3
"""
Grid Mapper for 3 ZED Cameras
Subscribes to obstacle detections from detector_3cam and builds a 2D occupancy grid
Publishes as nav_msgs/OccupancyGrid for visualization and path planning
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from exodus_interfaces.msg import ObstacleDetection
from geometry_msgs.msg import Pose
import numpy as np
import math

class GridMapper(Node):
    def __init__(self):
        super().__init__('grid_mapper_3cam')

        # Grid parameters (configurable)
        self.declare_parameter('grid_resolution', 0.1)  # 10cm per cell
        self.declare_parameter('grid_width', 20.0)      # 20m wide
        self.declare_parameter('grid_height', 20.0)     # 20m tall
        self.declare_parameter('obstacle_radius', 0.3)  # 30cm obstacle inflation
        self.declare_parameter('decay_rate', 0.95)      # Gradual decay of old obstacles

        self.resolution = self.get_parameter('grid_resolution').value
        self.width = self.get_parameter('grid_width').value
        self.height = self.get_parameter('grid_height').value
        self.obstacle_radius = self.get_parameter('obstacle_radius').value
        self.decay_rate = self.get_parameter('decay_rate').value

        # Calculate grid dimensions
        self.grid_width_cells = int(self.width / self.resolution)
        self.grid_height_cells = int(self.height / self.resolution)

        # Initialize occupancy grid (0 = free, 100 = occupied, -1 = unknown)
        # Start with unknown
        self.grid_data = np.full((self.grid_height_cells, self.grid_width_cells), -1, dtype=np.int8)

        # Initialize probability grid for better tracking (0.0 to 1.0)
        self.prob_grid = np.full((self.grid_height_cells, self.grid_width_cells), 0.5, dtype=np.float32)

        # Subscribe to obstacle detections from all 3 cameras
        self.sub_cam1 = self.create_subscription(
            ObstacleDetection,
            '/autonomy/obstacle_detections',
            self.detection_callback,
            10
        )

        # Publisher for occupancy grid
        self.grid_pub = self.create_publisher(
            OccupancyGrid,
            '/autonomy/occupancy_grid',
            10
        )

        # Timer to publish grid at 2 Hz
        self.timer = self.create_timer(0.5, self.publish_grid)

        # Timer to decay old obstacles at 0.5 Hz
        self.decay_timer = self.create_timer(2.0, self.decay_obstacles)

        self.get_logger().info(f'Grid Mapper initialized: {self.grid_width_cells}x{self.grid_height_cells} cells')
        self.get_logger().info(f'Resolution: {self.resolution}m, Size: {self.width}x{self.height}m')

    def world_to_grid(self, x, y):
        """Convert world coordinates (meters) to grid cell coordinates"""
        # Grid center is at robot position (0, 0)
        # X-axis points forward, Y-axis points left
        grid_x = int((x + self.width / 2) / self.resolution)
        grid_y = int((y + self.height / 2) / self.resolution)

        # Clamp to grid bounds
        grid_x = max(0, min(self.grid_width_cells - 1, grid_x))
        grid_y = max(0, min(self.grid_height_cells - 1, grid_y))

        return grid_x, grid_y

    def detection_callback(self, msg):
        """Process obstacle detections and update grid"""
        frame_id = msg.header.frame_id

        for detection in msg.detections:
            # Use distance_2d and assume obstacle is in front of camera
            distance = detection.distance_2d

            # Simple camera positioning based on frame_id
            # Adjust these based on your actual camera mounting positions
            if frame_id == "camera1_left_camera_frame":  # Front camera
                x = distance  # Forward
                y = 0.0
            elif frame_id == "camera2_left_camera_frame":  # Rear camera
                x = -distance  # Behind robot
                y = 0.0
            elif frame_id == "camera3_left_camera_frame":  # Manipulator camera
                x = distance  # Forward (adjust based on mounting)
                y = 0.0
            else:
                continue

            # Mark obstacle in grid with inflation radius
            self.mark_obstacle(x, y, detection.confidence)

    def mark_obstacle(self, x, y, confidence):
        """Mark an obstacle in the grid with inflation"""
        center_x, center_y = self.world_to_grid(x, y)

        # Calculate inflation radius in cells
        radius_cells = int(self.obstacle_radius / self.resolution)

        # Mark cells within radius as occupied
        for dx in range(-radius_cells, radius_cells + 1):
            for dy in range(-radius_cells, radius_cells + 1):
                grid_x = center_x + dx
                grid_y = center_y + dy

                # Check bounds
                if 0 <= grid_x < self.grid_width_cells and 0 <= grid_y < self.grid_height_cells:
                    distance = math.sqrt(dx**2 + dy**2) * self.resolution

                    if distance <= self.obstacle_radius:
                        # Increase probability based on confidence and distance
                        prob_increase = confidence * (1.0 - distance / self.obstacle_radius) * 0.3
                        self.prob_grid[grid_y, grid_x] = min(1.0, self.prob_grid[grid_y, grid_x] + prob_increase)

    def decay_obstacles(self):
        """Gradually decay obstacle probabilities over time"""
        # Decay towards 0.5 (unknown)
        self.prob_grid = self.prob_grid * self.decay_rate + 0.5 * (1 - self.decay_rate)

    def publish_grid(self):
        """Publish the occupancy grid"""
        # Convert probability grid to occupancy values
        # prob < 0.3 = free (0), prob > 0.7 = occupied (100), else unknown (-1)
        occupancy = np.where(self.prob_grid < 0.3, 0,
                            np.where(self.prob_grid > 0.7, 100, -1))

        # Create OccupancyGrid message
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = "base_link"

        grid_msg.info.resolution = self.resolution
        grid_msg.info.width = self.grid_width_cells
        grid_msg.info.height = self.grid_height_cells

        # Origin is at bottom-left corner of grid
        grid_msg.info.origin = Pose()
        grid_msg.info.origin.position.x = -self.width / 2
        grid_msg.info.origin.position.y = -self.height / 2
        grid_msg.info.origin.position.z = 0.0
        grid_msg.info.origin.orientation.w = 1.0

        # Flatten grid data (row-major order)
        grid_msg.data = occupancy.astype(np.int8).flatten().tolist()

        self.grid_pub.publish(grid_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GridMapper()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
