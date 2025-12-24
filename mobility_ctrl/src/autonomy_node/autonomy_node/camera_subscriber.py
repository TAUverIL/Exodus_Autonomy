#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge
import cv2
import numpy as np 
from ultralytics import YOLO 
import message_filters

# --- Custom ROS 2 Interface Imports ---
# These messages are defined in the exodus_interfaces package
from exodus_interfaces.msg import ObstacleDetection, Obstacle
from geometry_msgs.msg import Point
from std_msgs.msg import Header

# Define the exact ZED topic names found via 'ros2 topic list'
FRONT_RGB_TOPIC = '/zed_front/zed_node/rgb/color/rect/image'
FRONT_DEPTH_TOPIC = '/zed_front/zed_node/depth/depth_registered'
REAR_RGB_TOPIC = '/zed_rear/zed_node/rgb/color/rect/image'
REAR_DEPTH_TOPIC = '/zed_rear/zed_node/depth/depth_registered'

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.bridge = CvBridge()
        
        # 1. YOLO Model Loading
        try:
            # Assumes yolov8n.pt is accessible in the environment or path
            self.yolo_model = YOLO('yolov8n.pt') 
            self.get_logger().info('YOLOv8n model loaded successfully.')
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO model: {e}')
            self.yolo_model = None 
            
        # 2. Publisher Setup for Obstacle Data
        self.publisher_ = self.create_publisher(ObstacleDetection, '/autonomy/obstacle_detections', 10)

        # 3. Dual Camera Time Synchronization Setup
        
        # --- FRONT CAMERA SETUP ---
        sub_front_rgb = message_filters.Subscriber(self, Image, FRONT_RGB_TOPIC)
        sub_front_depth = message_filters.Subscriber(self, Image, FRONT_DEPTH_TOPIC)
        
        # ApproximateTimeSynchronizer is used because ZED RGB and Depth timestamps may not match exactly
        self.ts_front = message_filters.ApproximateTimeSynchronizer(
            [sub_front_rgb, sub_front_depth], queue_size=10, slop=0.05)
        self.ts_front.registerCallback(self.front_sync_callback)

        # --- REAR CAMERA SETUP ---
        sub_rear_rgb = message_filters.Subscriber(self, Image, REAR_RGB_TOPIC)
        sub_rear_depth = message_filters.Subscriber(self, Image, REAR_DEPTH_TOPIC)

        self.ts_rear = message_filters.ApproximateTimeSynchronizer(
            [sub_rear_rgb, sub_rear_depth], queue_size=10, slop=0.05)
        self.ts_rear.registerCallback(self.rear_sync_callback)
            
        self.get_logger().info('Autonomy Node: YOLO + Depth + Publishing Initialized.')

    def front_sync_callback(self, rgb_msg, depth_msg):
        """Callback for synchronized front camera data."""
        self.process_synchronized_data(rgb_msg, depth_msg, "zed_front_camera_link", "ZED Front View")

    def rear_sync_callback(self, rgb_msg, depth_msg):
        """Callback for synchronized rear camera data."""
        self.process_synchronized_data(rgb_msg, depth_msg, "zed_rear_camera_link", "ZED Rear View")

    def process_synchronized_data(self, rgb_msg, depth_msg, frame_id, window_name):
        """Main processing function for synchronized RGB and Depth frames."""
        if self.yolo_model is None:
            return 
            
        try:
            # Convert ROS messages to OpenCV images
            cv_image = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            # Depth image is 32-bit floating point (32FC1) in meters
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1") 
            
            # 1. Run YOLO inference
            results = self.yolo_model(cv_image, verbose=False) 
            
            # Prepare the custom ObstacleDetection message
            detection_msg = ObstacleDetection()
            detection_msg.header = Header()
            detection_msg.header.stamp = self.get_clock().now().to_msg()
            detection_msg.header.frame_id = frame_id # Use the TF frame ID

            annotated_frame = cv_image
            
            for r in results:
                # YOLO's r.plot() draws bounding boxes and labels
                annotated_frame = r.plot()
                
                # 2. Process detections, calculate depth, and populate ROS message
                self.process_detections(r.boxes, annotated_frame, depth_image, detection_msg)

            # 3. Publish the completed message (if objects were found)
            if len(detection_msg.detections) > 0:
                self.publisher_.publish(detection_msg)

            # 4. Display the image
            cv2.imshow(window_name, annotated_frame)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Error processing frame in {frame_id}: {e}')

    def process_detections(self, boxes, annotated_frame, depth_image, detection_msg):
        """Calculates depth and appends detection data to the ROS message."""
        if len(boxes) == 0:
            return

        for box in boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            
            # Clamp coordinates to ensure they are within the image boundaries
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(depth_image.shape[1], x2), min(depth_image.shape[0], y2)

            if x2 <= x1 or y2 <= y1:
                continue

            # Calculate Median Depth within the bounding box ROI
            depth_roi = depth_image[y1:y2, x1:x2]
            # Filter out infinite or NaN values (invalid depth readings)
            valid_depths = depth_roi[np.isfinite(depth_roi)]
            
            if valid_depths.size > 0:
                median_distance = float(np.median(valid_depths))
                label_idx = int(box.cls[0].item())
                label_name = self.yolo_model.names[label_idx]
                conf = float(box.conf[0].item())

                # --- Create and Populate Obstacle Message ---
                obs = Obstacle()
                obs.class_label = label_name
                obs.confidence = conf
                obs.distance_2d = median_distance
                
                # Approximate 3D Position (Z is distance, X/Y are placeholders for now)
                # True X/Y requires camera matrix and complex projection math
                obs.position_3d = Point()
                obs.position_3d.x = 0.0 
                obs.position_3d.y = 0.0
                obs.position_3d.z = median_distance 
                
                # Add the single obstacle to the list of detections
                detection_msg.detections.append(obs)
                
                # Update Visualization text on the OpenCV frame
                text = f"{label_name}: {median_distance:.2f}m"
                cv2.putText(annotated_frame, text, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)


# --- ROS 2 Python Entry Point ---
def main(args=None):
    rclpy.init(args=args)
    node = CameraSubscriber()
    
    try:
        # Keep the node running
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Handle graceful shutdown on Ctrl+C
        pass
    finally:
        # Clean up resources
        node.destroy_node()
        cv2.destroyAllWindows() 
        rclpy.shutdown()

if __name__ == '__main__':
    main()