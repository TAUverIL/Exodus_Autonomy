#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2 # For the fused 360 pointcloud

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        
        # Publisher for the fused 360 pointcloud (REQ-AUT-010)
        self._pointcloud_publisher = self.create_publisher(
            PointCloud2, 
            'perception/pointcloud', 
            10
        )
        
        # Timer to simulate or trigger perception logic
        self.timer = self.create_timer(0.1, self.perception_timer_cb) # 10 Hz
        
        self.get_logger().info('Perception Node initialized, publishing to /perception/pointcloud.')

    def perception_timer_cb(self):
        """
        Main callback to read sensor data, fuse pointclouds (from 4 stereo cameras 
        in sim), and publish the result.
        """
        
        # NOTE: Actual logic to read depth sensors, transform, and fuse pointclouds goes here.

        # Create a placeholder PointCloud2 message
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link' 
        # msg.height, msg.width, msg.fields, msg.data would be populated by the fusion algorithm

        # self._pointcloud_publisher.publish(msg)
        # Uncomment above line when message population is implemented

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()