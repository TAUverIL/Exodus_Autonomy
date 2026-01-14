#!/usr/bin/env python3
"""
A* Path Planner
Computes optimal path from current pose to goal using occupancy grid
Publishes path as waypoints to computer team
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped, Point
from exodus_interfaces.msg import WaypointArray, Waypoint  # Assuming custom message
import numpy as np
import heapq
import math

class AStarPlanner(Node):
    def __init__(self):
        super().__init__('astar_path_planner')

        # Current map and odometry
        self.occupancy_grid = None
        self.current_pose = None
        self.goal_pose = None

        # Subscribe to occupancy grid
        self.sub_grid = self.create_subscription(
            OccupancyGrid,
            '/autonomy/occupancy_grid',
            self.grid_callback,
            10
        )

        # Subscribe to fused odometry
        self.sub_odom = self.create_subscription(
            Odometry,
            '/autonomy/fused_odom',
            self.odom_callback,
            10
        )

        # Subscribe to goal pose
        self.sub_goal = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )

        # Publisher for path (for visualization)
        self.path_pub = self.create_publisher(
            Path,
            '/autonomy/planned_path',
            10
        )

        # Publisher for waypoints (to computer team)
        # Note: Using Path message - adjust if computer team needs different format
        self.waypoint_pub = self.create_publisher(
            Path,
            '/autonomy/waypoints',
            10
        )

        self.get_logger().info('A* Path Planner initialized')
        self.get_logger().info('Waiting for occupancy grid, odometry, and goal...')

    def grid_callback(self, msg):
        """Store latest occupancy grid"""
        self.occupancy_grid = msg

    def odom_callback(self, msg):
        """Store current pose from odometry"""
        self.current_pose = msg.pose.pose

    def goal_callback(self, msg):
        """Received new goal - plan path"""
        self.goal_pose = msg.pose
        self.get_logger().info(f'New goal received: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')

        # Plan path
        self.plan_path()

    def world_to_grid(self, x, y, grid_msg):
        """Convert world coordinates to grid cell indices"""
        origin_x = grid_msg.info.origin.position.x
        origin_y = grid_msg.info.origin.position.y
        resolution = grid_msg.info.resolution

        grid_x = int((x - origin_x) / resolution)
        grid_y = int((y - origin_y) / resolution)

        return grid_x, grid_y

    def grid_to_world(self, grid_x, grid_y, grid_msg):
        """Convert grid cell indices to world coordinates"""
        origin_x = grid_msg.info.origin.position.x
        origin_y = grid_msg.info.origin.position.y
        resolution = grid_msg.info.resolution

        x = origin_x + (grid_x + 0.5) * resolution
        y = origin_y + (grid_y + 0.5) * resolution

        return x, y

    def is_valid_cell(self, x, y, grid_msg):
        """Check if cell is within bounds and not occupied"""
        width = grid_msg.info.width
        height = grid_msg.info.height

        if x < 0 or x >= width or y < 0 or y >= height:
            return False

        idx = y * width + x
        cell_value = grid_msg.data[idx]

        # Free if value is 0 or unknown (-1)
        # Occupied if value is 100
        return cell_value != 100

    def heuristic(self, x1, y1, x2, y2):
        """Euclidean distance heuristic"""
        return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

    def astar(self, start_x, start_y, goal_x, goal_y, grid_msg):
        """A* pathfinding algorithm"""
        width = grid_msg.info.width
        height = grid_msg.info.height

        # Priority queue: (f_score, counter, (x, y))
        counter = 0
        open_set = [(0, counter, (start_x, start_y))]
        counter += 1

        # Track visited cells
        came_from = {}

        # Cost from start to each cell
        g_score = {(start_x, start_y): 0}

        # Estimated total cost
        f_score = {(start_x, start_y): self.heuristic(start_x, start_y, goal_x, goal_y)}

        # 8-connected neighbors
        neighbors = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]

        while open_set:
            _, _, current = heapq.heappop(open_set)

            # Goal reached
            if current == (goal_x, goal_y):
                return self.reconstruct_path(came_from, current)

            curr_x, curr_y = current

            for dx, dy in neighbors:
                neighbor = (curr_x + dx, curr_y + dy)

                if not self.is_valid_cell(neighbor[0], neighbor[1], grid_msg):
                    continue

                # Cost to move to neighbor (diagonal = sqrt(2), straight = 1)
                move_cost = math.sqrt(dx**2 + dy**2)
                tentative_g_score = g_score[current] + move_cost

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor[0], neighbor[1], goal_x, goal_y)

                    heapq.heappush(open_set, (f_score[neighbor], counter, neighbor))
                    counter += 1

        # No path found
        return None

    def reconstruct_path(self, came_from, current):
        """Reconstruct path from goal to start"""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def simplify_path(self, path, grid_msg):
        """Simplify path by removing unnecessary waypoints using Douglas-Peucker"""
        if len(path) <= 2:
            return path

        # Simple approach: take every Nth point
        simplified = [path[0]]
        step = max(1, len(path) // 10)  # ~10 waypoints

        for i in range(step, len(path), step):
            simplified.append(path[i])

        if simplified[-1] != path[-1]:
            simplified.append(path[-1])

        return simplified

    def plan_path(self):
        """Plan path from current pose to goal using A*"""
        if self.occupancy_grid is None:
            self.get_logger().warn('No occupancy grid available')
            return

        if self.current_pose is None:
            self.get_logger().warn('No current pose available')
            return

        if self.goal_pose is None:
            self.get_logger().warn('No goal pose available')
            return

        # Convert poses to grid coordinates
        start_x, start_y = self.world_to_grid(
            self.current_pose.position.x,
            self.current_pose.position.y,
            self.occupancy_grid
        )

        goal_x, goal_y = self.world_to_grid(
            self.goal_pose.position.x,
            self.goal_pose.position.y,
            self.occupancy_grid
        )

        self.get_logger().info(f'Planning from ({start_x}, {start_y}) to ({goal_x}, {goal_y})')

        # Run A*
        path = self.astar(start_x, start_y, goal_x, goal_y, self.occupancy_grid)

        if path is None:
            self.get_logger().warn('No path found to goal!')
            return

        self.get_logger().info(f'Path found with {len(path)} cells')

        # Simplify path
        simplified_path = self.simplify_path(path, self.occupancy_grid)
        self.get_logger().info(f'Simplified to {len(simplified_path)} waypoints')

        # Convert to Path message
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = self.occupancy_grid.header.frame_id

        for grid_x, grid_y in simplified_path:
            world_x, world_y = self.grid_to_world(grid_x, grid_y, self.occupancy_grid)

            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = world_x
            pose.pose.position.y = world_y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0

            path_msg.poses.append(pose)

        # Publish for visualization
        self.path_pub.publish(path_msg)

        # Publish waypoints to computer team
        self.waypoint_pub.publish(path_msg)

        self.get_logger().info('Path published!')

def main(args=None):
    rclpy.init(args=args)
    node = AStarPlanner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
