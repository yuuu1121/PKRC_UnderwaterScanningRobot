from setuptools import find_packages, setup

package_name = 'active_marker'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Active ArUco Marker Detector - supports up to 30 markers (ID 0-29)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_detector_6dof = active_marker.aruco_detector_6dof:main',
            'stellar_camera_publisher = active_marker.stellar_camera_publisher:main',
        ],
    },
)
