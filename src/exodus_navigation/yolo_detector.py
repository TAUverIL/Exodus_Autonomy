import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from ultralytics import YOLO
import cv2

class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        
        # Parameters
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        # Default to ZED2 left camera topic
        self.declare_parameter('image_topic', '/zed2/left/image_rect_color')
        
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.conf_thresh = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value

        self.get_logger().info(f'Loading YOLO model: {model_path}')
        # Load the YOLOv8 model (will download automatically if not present)
        self.model = YOLO(model_path)
        
        self.bridge = CvBridge()
        
        # Subscribe to the camera image
        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10)
            
        # Publisher for detection results (standard vision_msgs)
        self.detection_pub = self.create_publisher(Detection2DArray, '/detections', 10)
        # Publisher for debug image with bounding boxes drawn
        self.debug_pub = self.create_publisher(Image, '/detections_debug', 10)

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CV Bridge error: {str(e)}')
            return

        # Run YOLO inference
        results = self.model(cv_image, verbose=False, conf=self.conf_thresh)
        
        # Prepare the output message
        det_msg = Detection2DArray()
        det_msg.header = msg.header
        
        # Process detections
        for r in results:
            boxes = r.boxes
            for box in boxes:
                # Create Detection2D message
                detection = Detection2D()
                detection.header = msg.header
                
                # Bounding box (x, y, w, h)
                x, y, w, h = box.xywh[0]
                detection.bbox.center.x = float(x)
                detection.bbox.center.y = float(y)
                detection.bbox.size_x = float(w)
                detection.bbox.size_y = float(h)
                
                # Class hypothesis (what object is it?)
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = self.model.names[int(box.cls)]
                hypothesis.hypothesis.score = float(box.conf)
                detection.results.append(hypothesis)
                
                det_msg.detections.append(detection)
                
            # Create debug image with boxes drawn
            annotated_frame = r.plot()
            debug_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
            self.debug_pub.publish(debug_msg)

        # Publish results
        self.detection_pub.publish(det_msg)

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
