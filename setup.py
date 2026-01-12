from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'lekiwi_ros2_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='inoma',
    maintainer_email='inoma@users.noreply.github.com',
    description='ROS2 teleoperation node for LeKiwi robot',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'lekiwi_teleop_node = lekiwi_ros2_teleop.lekiwi_teleop_node:main',
            'lekiwi_vla_node = lekiwi_ros2_teleop.lekiwi_vla_node:main',
            'lekiwi_ros2_teleop_client = lekiwi_ros2_teleop.lekiwi_ros2_teleop_client:main',
            'lekiwi_data_recorder = lekiwi_ros2_teleop.lekiwi_data_recorder:main',
            'lekiwi_upload_dataset = lekiwi_ros2_teleop.upload_dataset:main',
        ],
    },
)
