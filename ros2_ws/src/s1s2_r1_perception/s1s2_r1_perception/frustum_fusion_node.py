import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from vision_msgs.msg import Detection3DArray, Detection2DArray, Detection3D, ObjectHypothesisWithPose
from geometry_msgs.msg import PointStamped
import tf2_ros
import tf2_geometry_msgs
import numpy as np
import message_filters

class FrustumFusionNode(Node):
    def __init__(self):
        super().__init__('frustum_fusion_node')
        
        # Parameters
        self.declare_parameter('lidar_frame', 'rslidar')
        self.declare_parameter('camera_frame', 'zed2_left_camera_optical_frame')
        self.declare_parameter('match_radius', 150.0) # Search radius in pixels
        
        self.lidar_frame = self.get_parameter('lidar_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.match_radius = self.get_parameter('match_radius').value

        # TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Camera Info Subscription
        self.info_sub = self.create_subscription(
            CameraInfo,
            '/zed2/zed/rgb/camera_info',
            self.info_callback,
            10)
        self.camera_info = None
        self.K = None
        self.K = np.array([
            [267.0,   0.0, 320.0],
            [  0.0, 267.0, 180.0],
            [  0.0,   0.0,   1.0]
        ])

        # Synchronized Subscriptions
        self.lidar_sub = message_filters.Subscriber(self, Detection3DArray, '/detection/lidar_clusters')
        self.camera_sub = message_filters.Subscriber(self, Detection2DArray, '/detection/camera_2d')
        
        # Use approximate sync since timestamps might not be identical in bag files
        self.sync = message_filters.ApproximateTimeSynchronizer([self.lidar_sub, self.camera_sub], 10, 0.1)
        self.sync.registerCallback(self.fused_callback)

        # Publisher
        self.fused_pub = self.create_publisher(Detection3DArray, '/detection/fused_3d', 10)
        
        self.get_logger().info('Frustum Fusion Node started.')

    def info_callback(self, msg):
        self.camera_info = msg
        self.K = np.array(msg.k).reshape((3, 3))

    def fused_callback(self, lidar_msg, camera_msg):
        if self.K is None:
            self.get_logger().warn('No camera info received yet.')
            return

        # Look up transform from Lidar to Camera
        try:
            transform = self.tf_buffer.lookup_transform(
                self.camera_frame,
                lidar_msg.header.frame_id,
                lidar_msg.header.stamp,
                rclpy.duration.Duration(seconds=0.1))
        except Exception as e:
            self.get_logger().error(f'Could not lookup transform: {e}')
            return

        fused_detections = Detection3DArray()
        fused_detections.header = lidar_msg.header

        for det_3d in lidar_msg.detections:
            # 1. Transform centroid to camera frame
            point_lidar = PointStamped()
            point_lidar.header = lidar_msg.header
            point_lidar.point = det_3d.bbox.center.position
            
            point_cam = tf2_geometry_msgs.do_transform_point(point_lidar, transform)
            
            # Check if point is in front of camera
            if point_cam.point.z <= 0:
                continue
                
            # 2. Project onto 2D plane
            p_cam = np.array([point_cam.point.x, point_cam.point.y, point_cam.point.z])
            p_img = self.K @ p_cam
            u = p_img[0] / p_img[2]
            v = p_img[1] / p_img[2]

            # 3. Match with 2D detections
            best_match = None
            min_dist = self.match_radius
            
            for det_2d in camera_msg.detections:
                u_2d = det_2d.bbox.center.position.x
                v_2d = det_2d.bbox.center.position.y
                
                dist = np.sqrt((u - u_2d)**2 + (v - v_2d)**2)
                if dist < min_dist:
                    min_dist = dist
                    best_match = det_2d

            # 4. Update 3D detection with class if matched
            if best_match is not None:
                det_fused = det_3d # Copy 3D detection
                # Map YOLO class results
                for res_2d in best_match.results:
                    hypothesis = ObjectHypothesisWithPose()
                    hypothesis.hypothesis.class_id = res_2d.hypothesis.class_id
                    hypothesis.hypothesis.score = res_2d.hypothesis.score
                    det_fused.results.append(hypothesis)
                fused_detections.detections.append(det_fused)

        self.fused_pub.publish(fused_detections)

def main(args=None):
    rclpy.init(args=args)
    node = FrustumFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
