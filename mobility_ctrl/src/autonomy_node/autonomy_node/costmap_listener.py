#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid

class CostmapListener(Node):
    def __init__(self):
        super().__init__('costmap_listener')
        
        # TARGET TOPIC: Change this if your topic name is different
        # Common RTAB-Map topics: '/rtabmap/grid_map', '/map', '/rtabmap/proj_map'
        topic_name = '/rtabmap/map'
        
        # Maps are typically published with TRANSIENT_LOCAL durability (latched).
        # We must match this QoS to receive the data.
        qos_profile = QoSProfile(depth=10)
        qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.subscription = self.create_subscription(
            OccupancyGrid,
            topic_name,
            self.listener_callback,
            qos_profile)
        self.subscription  # prevent unused variable warning
        self.get_logger().info(f'Listening for costmap on: {topic_name}...')

    def listener_callback(self, msg):
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin = msg.info.origin.position
        
        self.get_logger().info(
            f'Received Map! Size: {width}x{height}, Res: {resolution:.3f}, Origin: ({origin.x:.2f}, {origin.y:.2f})'
        )

def main(args=None):
    rclpy.init(args=args)
    node = CostmapListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
