from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'jetson_layer'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'systemd'), glob('systemd/*.service')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='MINEBOT Team',
    maintainer_email='dev@minebot.io',
    description='MINEBOT-Q Jetson Orin NX Layer',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = nodes.camera_node:main',
            'lidar_node = nodes.lidar_node:main',
            'go2_bridge_node = nodes.go2_bridge_node:main',
            'state_publisher_node = nodes.state_publisher_node:main',
        ],
    },
)
