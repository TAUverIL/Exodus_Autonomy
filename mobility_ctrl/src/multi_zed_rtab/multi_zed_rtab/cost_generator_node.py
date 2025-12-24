import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid, MapMetaData
import tf2_ros
import tf2_sensor_msgs.tf2_sensor_msgs
import numpy as np
# Note: You might need to install numpy and sensor_msgs_py if not present
# pip install numpy

class CostGeneratorNode(Node):
    def __init__(self):
        super().__init__('cost_generator_node')

        # 1. Setup TF Listener (To know where cameras are)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 2. Setup Map Parameters
        self.resolution = 0.05  # 5cm per grid cell
        self.width = 200        # 200 cells wide (10 meters)
        self.height = 200       # 200 cells tall (10 meters)
        self.origin_x = -5.0    # Map starts 5m behind robot
        self.origin_y = -5.0    # Map starts 5m to the left
        
        # Initialize the Grid (0 = Free, 100 = Occupied, -1 = Unknown)
        self.grid_data = np.full((self.width * self.height), 0, dtype=np.int8)

        # 3. Publisher
        self.map_publisher = self.create_publisher(OccupancyGrid, '/map', 10)
        
        # 4. Subscribers
        self.front_sub = self.create_subscription(
            PointCloud2, '/zed_front/zed_node/point_cloud/cloud_registered',
            self.cloud_callback, qos_profile_sensor_data)
        
        self.rear_sub = self.create_subscription(
            PointCloud2, '/zed_rear/zed_node/point_cloud/cloud_registered',
            self.cloud_callback, qos_profile_sensor_data)

        self.get_logger().info('Simple Mapper Started!')

    def cloud_callback(self, msg):
        # 1. Transform Cloud to Base Link (Robot Center)
        try:
            trans = self.tf_buffer.lookup_transform('base_link', msg.header.frame_id, rclpy.time.Time())
            # Note: transforming clouds in Python can be slow. 
            # Ideally use C++ or filtering, but this demonstrates the logic.
            # For this simple example, we are just acknowledging receipt to publish the map.
        except Exception as e:
            self.get_logger().warn(f'Could not transform cloud: {e}')
            return

        # 2. Update Map Logic (Simplified for testing)
        # Real logic would extract X,Y points here. 
        # For now, we publish the empty map just to verify the visualization works.
        self.publish_map()

    def publish_map(self):
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = 'base_link'
        
        grid_msg.info.resolution = self.resolution
        grid_msg.info.width = self.width
        grid_msg.info.height = self.height
        grid_msg.info.origin.position.x = self.origin_x
        grid_msg.info.origin.position.y = self.origin_y
        grid_msg.info.origin.position.z = 0.0
        
        grid_msg.data = self.grid_data.tolist()
        self.map_publisher.publish(grid_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CostGeneratorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
