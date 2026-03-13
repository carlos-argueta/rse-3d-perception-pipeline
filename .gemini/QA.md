# Q&A Log

## Session: March 9, 2026

**Q1: Package Structure**
*Question:* Should I create a new ROS 2 package (e.g., `s1s2_r1_perception`) to house the four nodes and the integration launch file, or would you prefer them to be placed elsewhere?
*Answer:* Use a new ROS 2 package.

**Q2: Topic & Frame Confirmation**
*Question:* The instructions mention `/rslidar_points`, but the RViz configuration points to `/rslidar`. Which is the correct source? Also confirming camera topic `zed2/zed/rgb/image_rect_color` and frames (`rslidar`, `zed2_left_camera_optical_frame`).
*Answer:* Don't worry about the topic names (can be changed later). For TF frames, use the suggested ones (`rslidar`, `zed2_left_camera_optical_frame`).

**Q3: Perception Parameters**
*Question:* YOLOv8 Model size (Nano)? Default values for clustering/segmentation?
*Answer:* Option to switch between yolo8 and 11 nano. Propose best defaults for clustering/segmentation, to be tuned later.

**Q4: Input Source**
*Question:* Live simulation or ROS bag?
*Answer:* ROS bag, no simulator needed.

**Q5: Pixi Setup**
*Question:* Generate `pixi.toml` in root or subdirectory?
*Answer:* In the root is ok.

**Q6: Lidar Cluster Node Crash**
*Question:* The `lidar_cluster_node` crashes with `RuntimeError: There must be at least 'ransac_n' points.` during ground segmentation. How to fix this?
*Answer:* The crash was caused by `open3d.geometry.PointCloud.segment_plane` being called on a point cloud with fewer than 3 points (the `ransac_n` parameter). This can happen after downsampling if the input scan is sparse or the voxel size is too large. The fix involves adding a safety check for the point count after downsampling and before calling `segment_plane`, as well as filtering out non-finite points (NaN/Inf) during conversion. Performance was also improved by using `sensor_msgs_py.point_cloud2.read_points` instead of manual `struct` unpacking.

**Q7: PointCloud2 Conversion TypeError**
*Question:* The node crashes with `TypeError: Cannot cast array data from dtype(...) to dtype('float32')`. Why?
*Answer:* This occurs because `sensor_msgs_py.point_cloud2.read_points` returns a generator of structured data (e.g., named fields with specific offsets). Direct casting to `float32` via `np.array()` fails if the underlying structured dtype includes padding or doesn't match a flat array. The fix is to create a structured NumPy array first and then explicitly stack its named fields ('x', 'y', 'z') into a regular Nx3 array.

**Q8: Open3D AttributeError in PointCloud2 Publisher**
*Question:* The node crashes with `AttributeError: module 'open3d.cpu.pybind.io.rpc' has no attribute 'Field'`. How was this resolved?
*Answer:* Recent versions of Open3D have changed or removed parts of the `io.rpc` module. The redundant and broken manual field definition was removed, and the `numpy_to_pointcloud2` method was simplified to use the standard `sensor_msgs_py.point_cloud2.create_cloud_xyz32` directly, which is more robust and portable.
