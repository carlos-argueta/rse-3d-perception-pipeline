<img width="5120" height="2880" alt="image" src="https://github.com/user-attachments/assets/09832461-bfb8-45aa-8b9b-83f6b28ee508" />


# RSE 3D Perception Pipeline
This is the accompanying repository for the '𝗠𝗼𝘃𝗶𝗻𝗴 𝗳𝗿𝗼𝗺 𝟮𝗗 𝘁𝗼 𝟯𝗗: 𝗕𝘂𝗶𝗹𝗱𝗶𝗻𝗴 𝗮 𝗟𝗶𝗗𝗔𝗥-𝗖𝗮𝗺𝗲𝗿𝗮 𝗙𝘂𝘀𝗶𝗼𝗻 𝗣𝗶𝗽𝗲𝗹𝗶𝗻𝗲 𝗶𝗻 𝗥𝗢𝗦 𝟮' mini-project from the S1S2 Robotics Academy. You can access the course here: https://cm.s1s2.ai/spaces/23108951

The workspace contains a ROS 2 Jazzy 3D perception pipeline developed for the SHL-1 robot. The system processes Lidar point clouds and camera images to provide fused 3D object detections and a Bird's-Eye-View (BEV) visualization.




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
Download bag here: https://www.dropbox.com/scl/fi/tdxin6bzw01siucdv3kgv/linkou-2023-12-27-2-med.zip?rlkey=rcz93bhozjsdymcpn5dqz6rly&dl=0

Since this project is designed to run with recorded data, ensure you play your ROS bag with the `--clock` argument:
```bash
ros2 bag play <your_bag_file> --clock
```

## Packages

- **s1s2_r1_description**: Contains the URDF and meshes for the SHL-1 robot.
- **s1s2_r1_perception**: Implements the perception nodes (Lidar clustering, Camera detection, Frustum fusion, BEV visualization).

## License

MIT License
