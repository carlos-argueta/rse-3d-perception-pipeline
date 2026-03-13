from setuptools import find_packages, setup
import os
from glob import glob

package_name = 's1s2_r1_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='carlos',
    maintainer_email='c.argueta@s1s2.ai',
    description='Modular 3D Perception Pipeline',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_cluster_node = s1s2_r1_perception.lidar_cluster_node:main',
            'camera_detector_node = s1s2_r1_perception.camera_detector_node:main',
            'frustum_fusion_node = s1s2_r1_perception.frustum_fusion_node:main',
            'bev_visualizer_node = s1s2_r1_perception.bev_visualizer_node:main',
        ],
    },
)
