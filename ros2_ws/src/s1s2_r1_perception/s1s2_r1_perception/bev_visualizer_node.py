import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
from vision_msgs.msg import Detection3DArray
from cv_bridge import CvBridge
import numpy as np
import cv2
import struct

class BEVVisualizerNode(Node):
    def __init__(self):
        super().__init__('bev_visualizer_node')
        
        # BEV Parameters
        self.declare_parameter('range_x', 10.0) # +/- 10m
        self.declare_parameter('range_y', 10.0) # +/- 10m
        self.declare_parameter('resolution', 0.05) # 5cm/pixel
        
        self.range_x = self.get_parameter('range_x').value
        self.range_y = self.get_parameter('range_y').value
        self.resolution = self.get_parameter('resolution').value
        
        self.width = int(2 * self.range_y / self.resolution)
        self.height = int(2 * self.range_x / self.resolution)
        
        self.bridge = CvBridge()
        
        # Subscriptions
        self.lidar_sub = self.create_subscription(PointCloud2, '/rslidar_points', self.lidar_callback, 10)
        self.fused_sub = self.create_subscription(Detection3DArray, '/detection/fused_3d', self.fused_callback, 10)
        
        self.latest_fused = None
        
        # Publisher
        self.bev_pub = self.create_publisher(Image, '/visualization/bev_image', 10)
        
        self.get_logger().info(f'BEV Visualizer Node started. Image size: {self.width}x{self.height}')

    def fused_callback(self, msg):
        self.latest_fused = msg

    def lidar_callback(self, msg):
        # 1. Convert PointCloud2 to numpy
        points = self.pointcloud2_to_numpy(msg)
        if points.size == 0:
            return

        # 2. Filter points by range
        mask = (points[:, 0] > -self.range_x) & (points[:, 0] < self.range_x) & \
               (points[:, 1] > -self.range_y) & (points[:, 1] < self.range_y)
        points = points[mask]

        # 3. Project to pixels
        # OpenCV image coordinates: (row=y_pixel, col=x_pixel)
        # In BEV: x_lidar maps to row, y_lidar maps to col
        # Flip x to align with front-up
        x_pix = ((self.range_y - points[:, 1]) / self.resolution).astype(np.int32)
        y_pix = ((self.range_x - points[:, 0]) / self.resolution).astype(np.int32)

        # 4. Draw BEV
        bev_img = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        
        # Valid pixels
        valid_mask = (x_pix >= 0) & (x_pix < self.width) & (y_pix >= 0) & (y_pix < self.height)
        bev_img[y_pix[valid_mask], x_pix[valid_mask]] = [200, 200, 200]

        # 5. Draw Fused Detections
        if self.latest_fused:
            for det in self.latest_fused.detections:
                cx = det.bbox.center.position.x
                cy = det.bbox.center.position.y
                sx = det.bbox.size.x
                sy = det.bbox.size.y
                
                # Convert center to pixels
                px = int((self.range_y - cy) / self.resolution)
                py = int((self.range_x - cx) / self.resolution)
                
                # Convert size to pixels
                pw = int(sy / self.resolution)
                ph = int(sx / self.resolution)
                
                # Draw rectangle
                cv2.rectangle(bev_img, (px - pw//2, py - ph//2), (px + pw//2, py + ph//2), (0, 0, 255), 2)
                
                # Draw label
                if det.results:
                    cls_id = det.results[1].hypothesis.class_id
                    cv2.putText(bev_img, f"ID:{cls_id}", (px - pw//2, py - ph//2 - 5), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

        # 6. Publish BEV Image
        bev_msg = self.bridge.cv2_to_imgmsg(bev_img, encoding='bgr8')
        bev_msg.header = msg.header
        self.bev_pub.publish(bev_msg)

    def pointcloud2_to_numpy(self, msg):
        fmt = 'f' * 3
        point_step = msg.point_step
        data = msg.data
        points = []
        for i in range(0, len(data), point_step):
            try:
                x, y, z = struct.unpack_from('fff', data, i)
                points.append([x, y, z])
            except:
                continue
        return np.array(points, dtype=np.float32)

def main(args=None):
    rclpy.init(args=args)
    node = BEVVisualizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
