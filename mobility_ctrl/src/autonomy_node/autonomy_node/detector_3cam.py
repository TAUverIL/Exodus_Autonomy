#!/usr/bin/env python3
"""
UPDATED Camera Subscriber for 3 ZED Cameras
Subscribes to camera1, camera2, camera3 from your rqt_graph
Runs YOLO + Depth estimation
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import message_filters

from exodus_interfaces.msg import ObstacleDetection, Obstacle
from geometry_msgs.msg import Point
from std_msgs.msg import Header

class ThreeCameraDetector(Node):
    def __init__(self):
        super().__init__('three_camera_detector')
        self.bridge = CvBridge()

        # Load YOLO model
        try:
            self.yolo_model = YOLO('yolov8n.pt')
            self.get_logger().info('YOLOv8n model loaded successfully.')
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO model: {e}')
            self.yolo_model = None

        # Publisher for obstacle detections
        self.publisher_ = self.create_publisher(
            ObstacleDetection,
            '/autonomy/obstacle_detections',
            10
        )

        # Setup for 3 cameras based on your rqt_graph
        # Camera 1 topics
        sub_cam1_rgb = message_filters.Subscriber(
            self, Image, '/camera1/zed_node_1/rgb/image_rect_color'
        )
        sub_cam1_depth = message_filters.Subscriber(
            self, Image, '/camera1/zed_node_1/depth/depth_registered'
        )
        self.ts_cam1 = message_filters.ApproximateTimeSynchronizer(
            [sub_cam1_rgb, sub_cam1_depth], queue_size=10, slop=0.05
        )
        self.ts_cam1.registerCallback(
            lambda rgb, depth: self.process_camera(rgb, depth, "camera1", "Camera 1")
        )

        # Camera 2 topics
        sub_cam2_rgb = message_filters.Subscriber(
            self, Image, '/camera2/zed_node_2/rgb/image_rect_color'
        )
        sub_cam2_depth = message_filters.Subscriber(
            self, Image, '/camera2/zed_node_2/depth/depth_registered'
        )
        self.ts_cam2 = message_filters.ApproximateTimeSynchronizer(
            [sub_cam2_rgb, sub_cam2_depth], queue_size=10, slop=0.05
        )
        self.ts_cam2.registerCallback(
            lambda rgb, depth: self.process_camera(rgb, depth, "camera2", "Camera 2")
        )

        # Camera 3 topics (NEW!)
        sub_cam3_rgb = message_filters.Subscriber(
            self, Image, '/camera3/zed_node_3/rgb/image_rect_color'
        )
        sub_cam3_depth = message_filters.Subscriber(
            self, Image, '/camera3/zed_node_3/depth/depth_registered'
        )
        self.ts_cam3 = message_filters.ApproximateTimeSynchronizer(
            [sub_cam3_rgb, sub_cam3_depth], queue_size=10, slop=0.05
        )
        self.ts_cam3.registerCallback(
            lambda rgb, depth: self.process_camera(rgb, depth, "camera3", "Camera 3")
        )

        self.get_logger().info('3-Camera YOLO Detector initialized!')

    def process_camera(self, rgb_msg, depth_msg, frame_id, window_name):
        """Process synchronized RGB and Depth from one camera"""
        if self.yolo_model is None:
            return

        try:
            # Convert ROS messages to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1")

            # Run YOLO
            results = self.yolo_model(cv_image, verbose=False)

            # Create detection message
            detection_msg = ObstacleDetection()
            detection_msg.header = Header()
            detection_msg.header.stamp = self.get_clock().now().to_msg()
            detection_msg.header.frame_id = frame_id

            annotated_frame = cv_image

            for r in results:
                annotated_frame = r.plot()
                self.process_detections(
                    r.boxes, annotated_frame, depth_image, detection_msg
                )

            # Publish detections
            if len(detection_msg.detections) > 0:
                self.publisher_.publish(detection_msg)

            # Visualize (optional - can disable for performance)
            cv2.imshow(window_name, annotated_frame)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Error in {frame_id}: {e}')

    def process_detections(self, boxes, annotated_frame, depth_image, detection_msg):
        """Extract depth and create obstacle messages"""
        if len(boxes) == 0:
            return

        for box in boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])

            # Clamp to image bounds
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(depth_image.shape[1], x2), min(depth_image.shape[0], y2)

            if x2 <= x1 or y2 <= y1:
                continue

            # Get median depth in bbox
            depth_roi = depth_image[y1:y2, x1:x2]
            valid_depths = depth_roi[np.isfinite(depth_roi)]

            if valid_depths.size > 0:
                median_distance = float(np.median(valid_depths))
                label_idx = int(box.cls[0].item())
                label_name = self.yolo_model.names[label_idx]
                conf = float(box.conf[0].item())

                # Create obstacle message
                obs = Obstacle()
                obs.class_label = label_name
                obs.confidence = conf
                obs.distance_2d = median_distance

                obs.position_3d = Point()
                obs.position_3d.x = 0.0  # TODO: Calculate from camera intrinsics
                obs.position_3d.y = 0.0
                obs.position_3d.z = median_distance

                detection_msg.detections.append(obs)

                # Annotate
                text = f"{label_name}: {median_distance:.2f}m"
                cv2.putText(
                    annotated_frame, text, (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2
                )

def main(args=None):
    rclpy.init(args=args)
    node = ThreeCameraDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
