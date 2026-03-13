# RSE 3D Perception Pipeline

This workspace contains a ROS 2 Jazzy 3D perception pipeline developed for the SHL-1 robot. The system processes Lidar point clouds and camera images to provide fused 3D object detections and a Bird's-Eye-View (BEV) visualization.

## Prerequisites

- Ubuntu 24.04 (Noble Numbat)
- ROS 2 Jazzy Jalisco
- Pixi (Package Manager)

## Installation

1. Clone the repository (if applicable) or navigate to the workspace root.
2. Initialize the environment using Pixi:
   ```bash
   pixi install
   ```

## Build

Build the workspace using `colcon`:
```bash
pixi run build
source install/setup.bash
```

## Running the Pipeline

To launch the full perception pipeline:
```bash
ros2 launch s1s2_r1_perception perception_launch.py
```

### With ROS Bag

Since this project is designed to run with recorded data, ensure you play your ROS bag with the `--clock` argument:
```bash
ros2 bag play <your_bag_file> --clock
```

## Packages

- **s1s2_r1_description**: Contains the URDF and meshes for the SHL-1 robot.
- **s1s2_r1_perception**: Implements the perception nodes (Lidar clustering, Camera detection, Frustum fusion, BEV visualization).

## License

TODO: Add license.
