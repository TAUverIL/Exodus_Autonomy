from rclpy.node import Node # <--- MAKE SURE THIS IS PRESENT
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node # <--- MAKE SURE THIS IS PRESENT
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO 

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.bridge = CvBridge()

        # NEW: Load the YOLOv8 model (using 'n' for nano/fastest)
        self.yolo_model = YOLO('yolov8n.pt') 

        self.subscription = self.create_subscription(
            Image,
            '/zed_front/zed_node/rgb/color/rect/image', 
            self.listener_callback,
            10)
        # ... rest of __init__