import os
from glob import glob
from setuptools import setup

package_name = 'bar10xt_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'smbus2'],
    zip_safe=True,
    maintainer='hero',
    maintainer_email='kwbnoa1234@gmail.com',
    description='ROS2 (Humble) driver node for the Keller Bar10XT pressure sensor.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bar10xt_node = bar10xt_ros2.bar10xt_node:main',
            'bar10xt_ros_node = bar10xt_ros2.bar10xt_ros_node:main',
        ],
    },
)
