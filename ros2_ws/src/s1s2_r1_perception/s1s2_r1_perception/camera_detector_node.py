import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

class CameraDetectorNode(Node):
    def __init__(self):
        super().__init__('camera_detector_node')
        
        # Parameters
        self.declare_parameter('model_version', 'yolov8n') # Options: yolov8n, yolo11n
        self.declare_parameter('conf_threshold', 0.25)
        
        model_ver = self.get_parameter('model_version').value
        self.conf_threshold = self.get_parameter('conf_threshold').value
        
        # Load model (downloads if not present)
        model_path = f"{model_ver}.pt"
        self.model = YOLO(model_path)
        self.class_names = self.model.names
        
        self.bridge = CvBridge()
        
        # Subscription
        self.subscription = self.create_subscription(
            Image,
            'zed2/zed/rgb/image_rect_color',
            self.listener_callback,
            10)
        
        # Publisher
        self.detection_pub = self.create_publisher(Detection2DArray, '/detection/camera_2d', 10)
        self.annotated_pub = self.create_publisher(Image, '/detection/camera_annotated', 10)
        self.get_logger().info(f'Camera Detector Node started with model: {model_path}')

    def listener_callback(self, msg):
        # Convert ROS Image to OpenCV
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Could not convert image: {e}')
            return

        # Run Inference (CPU mode by default)
        results = self.model.predict(cv_image, conf=self.conf_threshold, verbose=False)

        # Prepare Detection2DArray
        detections = Detection2DArray()
        detections.header = msg.header
        
        for result in results:
            for box in result.boxes:
                # Extract box coordinates
                # box.xywh returns [x_center, y_center, width, height]
                xywh = box.xywh[0].cpu().numpy()
                cls_id = int(box.cls[0].cpu().item())
                conf = float(box.conf[0].cpu().item())
                
                detection = Detection2D()
                detection.header = msg.header
                
                # Bounding box
                detection.bbox.center.position.x = float(xywh[0])
                detection.bbox.center.position.y = float(xywh[1])
                detection.bbox.size_x = float(xywh[2])
                detection.bbox.size_y = float(xywh[3])
                
                # Hypothesis
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = self.class_names[cls_id]
                hypothesis.hypothesis.score = conf
                detection.results.append(hypothesis)
                
                detections.detections.append(detection)

        # Publish results
        self.detection_pub.publish(detections)
        
        # --- Visualize the results ---
        result = results[0]
        annotated_frame = result.plot()

        # --- Publish the annotated image ---
        try:
            # Convert the annotated OpenCV image back to a ROS Image message
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
            # Copy the header from the input message to keep the timestamp sync
            annotated_msg.header = msg.header
            # Publish the message
            self.annotated_pub.publish(annotated_msg)

        except Exception as e:
            self.get_logger().error(f'Failed to convert and publish annotated image: {e}')



def main(args=None):
    rclpy.init(args=args)
    node = CameraDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
