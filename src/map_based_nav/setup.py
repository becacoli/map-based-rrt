from setuptools import setup
import os
from glob import glob

package_name = 'map_based_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*')),
        ('share/' + package_name + '/models', glob('models/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='becacoli',
    maintainer_email='becacoli@example.com',
    description='RRT navigation for TurtleBot3 in Gazebo (ROS2 Jazzy)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'rrt_planner = map_based_nav.rrt_planner:main',
        ],
    },
)
