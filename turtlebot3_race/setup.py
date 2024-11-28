from setuptools import setup
import os
from glob import glob

package_name = 'turtlebot3_race'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/racetrack_launch.py')),
        # World files
        (os.path.join('share', package_name, 'worlds'), glob('worlds/hexagonal_track.world')),
        # Model files
        (os.path.join('share', package_name, 'models/turtlebot3_waffle'), glob('models/turtlebot3_waffle/turtlebot3_waffle.urdf.xacro')),
        # YOLOv8 model weights
        (os.path.join('share', package_name, 'yolo_weights'), ['resource/turtlebot3_yolov8n.pt']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='A ROS2 package for simulating a head-to-head TurtleBot race using YOLOv8 and depth camera.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_node = turtlebot3_race.yolo_node:main',
        ],
    },
)

