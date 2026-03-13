# s1s2_r1_perception

This package provides a modular 3D perception pipeline for the SHL-1 robot.

## Nodes

### 1. `lidar_cluster_node`

Subscribes to point clouds, segments the ground plane, clusters the remaining points, and publishes 3D bounding boxes.

- **Topic**: `/rslidar_points` (Input: `sensor_msgs/PointCloud2`)
- **Publishes**: 
    - `/detection/lidar_clusters` (`vision_msgs/Detection3DArray`)
    - `/lidar/points_no_ground` (`sensor_msgs/PointCloud2`)
- **Parameters**:
    - `voxel_size` (0.1): Leaf size for voxel grid downsampling.
    - `ground_distance_threshold` (0.2): Distance threshold for RANSAC plane segmentation.
    - `cluster_tolerance` (0.5): Epsilon distance for Euclidean clustering.
    - `min_cluster_size` (50): Minimum points for a valid cluster.
    - `max_cluster_size` (10000): Maximum points for a cluster.

### 2. `camera_detector_node`

Runs YOLOv8/11 object detection on camera images.

- **Topic**: `zed2/zed/rgb/image_rect_color` (Input: `sensor_msgs/Image`)
- **Publishes**: `/detection/camera_2d` (`vision_msgs/Detection2DArray`)
- **Parameters**:
    - `model_version` ('yolov8n'): Model to use ('yolov8n' or 'yolo11n').
    - `conf_threshold` (0.25): Confidence threshold for detections.

### 3. `frustum_fusion_node`

Fuses 3D lidar clusters with 2D camera detections by projecting 3D centroids onto the image plane.

- **Subscribes**: 
    - `/detection/lidar_clusters` (`vision_msgs/Detection3DArray`)
    - `/detection/camera_2d` (`vision_msgs/Detection2DArray`)
    - `/zed2/zed/rgb/camera_info` (`sensor_msgs/CameraInfo`)
- **Publishes**: `/detection/fused_3d` (`vision_msgs/Detection3DArray`)
- **Parameters**:
    - `lidar_frame` ('rslidar'): Source frame of the lidar data.
    - `camera_frame` ('zed2_left_camera_optical_frame'): Target camera optical frame.
    - `match_radius` (50.0): Pixel radius to match a projected 3D point to a 2D bounding box center.

### 4. `bev_visualizer_node`

Generates a top-down Bird's-Eye-View image visualizing the lidar points and fused detections.

- **Subscribes**: 
    - `/rslidar_points` (`sensor_msgs/PointCloud2`)
    - `/detection/fused_3d` (`vision_msgs/Detection3DArray`)
- **Publishes**: `/visualization/bev_image` (`sensor_msgs/Image`)
- **Parameters**:
    - `range_x` (10.0): View range in X direction (meters).
    - `range_y` (10.0): View range in Y direction (meters).
    - `resolution` (0.05): Meters per pixel.

## Launch

Use the provided launch file to start all nodes and the robot state publisher:

```bash
ros2 launch s1s2_r1_perception perception_launch.py
```
