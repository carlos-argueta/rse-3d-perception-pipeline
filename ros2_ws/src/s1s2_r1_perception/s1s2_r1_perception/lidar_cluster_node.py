import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from vision_msgs.msg import Detection3DArray, Detection3D, BoundingBox3D, ObjectHypothesisWithPose, ObjectHypothesis
from geometry_msgs.msg import Pose, PoseWithCovariance 
import open3d as o3d
import numpy as np

class LidarClusterNode(Node):
    def __init__(self):
        super().__init__('lidar_cluster_node')
        
        # Parameters (configurable via CLI or launch)
        self.declare_parameter('voxel_size', 0.05)
        self.declare_parameter('ground_distance_threshold', 0.5)
        self.declare_parameter('cluster_tolerance', 0.5)
        self.declare_parameter('min_cluster_size', 50)
        self.declare_parameter('max_cluster_size', 1000)
        
        self.voxel_size = self.get_parameter('voxel_size').value
        self.ground_dist = self.get_parameter('ground_distance_threshold').value
        self.cluster_tol = self.get_parameter('cluster_tolerance').value
        self.min_cluster = self.get_parameter('min_cluster_size').value
        self.max_cluster = self.get_parameter('max_cluster_size').value

        # Subscription
        self.subscription = self.create_subscription(
            PointCloud2,
            '/rslidar_points',
            self.listener_callback,
            10)
        
        # Publishers
        self.cluster_pub = self.create_publisher(Detection3DArray, '/detection/lidar_clusters', 10)
        self.no_ground_pub = self.create_publisher(PointCloud2, '/lidar/points_no_ground', 10)
        
        self.get_logger().info('Lidar Cluster Node started.')

    def listener_callback(self, msg):
        # Live-update parameters from the ROS 2 server
        self.voxel_size = self.get_parameter('voxel_size').value
        self.ground_dist = self.get_parameter('ground_distance_threshold').value
        self.cluster_tol = self.get_parameter('cluster_tolerance').value
        self.min_cluster = self.get_parameter('min_cluster_size').value
        self.max_cluster = self.get_parameter('max_cluster_size').value

        # 1. Convert ROS PointCloud2 to Open3D PointCloud
        points = self.pointcloud2_to_numpy(msg)
        if points.size == 0:
            return
        
        # self.get_logger().info(f'Received PointCloud with {points.shape[0]} points.')
        
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        # 2. Downsample (Voxel Grid)
        pcd_down = pcd.voxel_down_sample(voxel_size=self.voxel_size)

        # 3. Segment Ground (RANSAC)
        if len(pcd_down.points) < 3:
            self.get_logger().warn(f'Not enough points after downsampling: {len(pcd_down.points)}')
            return

        plane_model, inliers = pcd_down.segment_plane(distance_threshold=self.ground_dist,
                                                     ransac_n=3,
                                                     num_iterations=100)
        
        # Extract non-ground points
        pcd_no_ground = pcd_down.select_by_index(inliers, invert=True)
        if len(pcd_no_ground.points) == 0:
            return

        # 4. Extract Clusters (Euclidean Clustering)
        labels = np.array(pcd_no_ground.cluster_dbscan(eps=self.cluster_tol, min_points=self.min_cluster, print_progress=False))

        # self.get_logger().info(f'Found {labels.max() + 1} clusters (labels: {labels})')

        # Prepare Detection3DArray
        detections = Detection3DArray()
        detections.header = msg.header
        
        max_label = labels.max() if labels.size > 0 else -1
        
        for i in range(max_label + 1):
            cluster_indices = np.where(labels == i)[0]
            if len(cluster_indices) < self.min_cluster:
                continue
                
            cluster_pcd = pcd_no_ground.select_by_index(cluster_indices)
            
            # Compute AABB or OBB
            aabb = cluster_pcd.get_axis_aligned_bounding_box()
            center = aabb.get_center()
            extent = aabb.get_extent()
            
            detection = Detection3D()
            detection.header = msg.header

            hypothesis = ObjectHypothesisWithPose()

            hp = ObjectHypothesis()
            hp.class_id = f'cluster_{i}'
            hp.score = 1.0
            hypothesis.hypothesis = hp
            
            
            hypothesis.pose = PoseWithCovariance()
            hypothesis.pose.pose = Pose()
            hypothesis.pose.pose.position.x = center[0]
            hypothesis.pose.pose.position.y = center[1]
            hypothesis.pose.pose.position.z = center[2]
            hypothesis.pose.pose.orientation.x = 0.0
            hypothesis.pose.pose.orientation.y = 0.0
            hypothesis.pose.pose.orientation.z = 0.0
            hypothesis.pose.pose.orientation.w = 1.0
            detection.results.append(hypothesis)
            
            bbox = BoundingBox3D()
            bbox.center.position.x = center[0]
            bbox.center.position.y = center[1]
            bbox.center.position.z = center[2]
            bbox.size.x = extent[0]
            bbox.size.y = extent[1]
            bbox.size.z = extent[2]
            
            detection.bbox = bbox
            detections.detections.append(detection)

        # Publish results
        self.cluster_pub.publish(detections)
        
        # Publish No-Ground PointCloud for debugging
        no_ground_msg = self.numpy_to_pointcloud2(np.asarray(pcd_no_ground.points), msg.header)
        self.no_ground_pub.publish(no_ground_msg)

    def pointcloud2_to_numpy(self, msg):
        # 1. Create a generator for the points
        gen = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        
        # 2. Define the data type for the generator (3 floats for x, y, z)
        # We use a structured dtype to match the generator's output
        dtype = [('x', np.float32), ('y', np.float32), ('z', np.float32)]
        
        # 3. Use fromiter to pull data directly into NumPy without creating a Python list
        structured_array = np.fromiter(gen, dtype=dtype)
        
        # 4. Convert the structured array to a standard Nx3 float32 array
        # .view transforms the memory layout without copying the data
        if structured_array.size > 0:
            points = structured_array.view(np.float32).reshape(-1, 3)
            
            # 5. Final safety check for invalid numbers
            points = points[np.isfinite(points).all(axis=1)]
            return points
        
        return np.zeros((0, 3), dtype=np.float32)

    def numpy_to_pointcloud2(self, points, header):
        # Create PointCloud2 from numpy array (Nx3)
        return point_cloud2.create_cloud_xyz32(header, points)

def main(args=None):
    rclpy.init(args=args)
    node = LidarClusterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
