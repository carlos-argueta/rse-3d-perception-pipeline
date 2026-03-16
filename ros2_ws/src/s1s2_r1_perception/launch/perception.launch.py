import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro

def generate_launch_description():
    
    # 1. Robot State Publisher (from s1s2_r1_description)
    pkg_description = get_package_share_directory('s1s2_r1_description')
    xacro_file = os.path.join(pkg_description, 'urdf', 'r1_fixed.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = robot_description_config.toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }],
    )

    # 2. Lidar Cluster Node
    lidar_node = Node(
        package='s1s2_r1_perception',
        executable='lidar_cluster_node',
        name='lidar_cluster_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 3. Camera Detector Node
    camera_node = Node(
        package='s1s2_r1_perception',
        executable='camera_detector_node',
        name='camera_detector_node',
        output='screen',
        parameters=[{'use_sim_time': True, 'model_version': 'yolov8n'}]
    )

    # 4. Frustum Fusion Node
    fusion_node = Node(
        package='s1s2_r1_perception',
        executable='frustum_fusion_node',
        name='frustum_fusion_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 5. BEV Visualizer Node
    bev_node = Node(
        package='s1s2_r1_perception',
        executable='bev_visualizer_node',
        name='bev_visualizer_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 6. RViz2 Node
    # Get path to rviz config
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('s1s2_r1_description'),
        'rviz',
        'display.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}]
    )
    
    return LaunchDescription([
        robot_state_publisher,
        lidar_node,
        camera_node,
        fusion_node,
        bev_node,
        rviz_node
    ])
