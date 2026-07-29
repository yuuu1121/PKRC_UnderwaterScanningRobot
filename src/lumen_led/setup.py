from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'lumen_led'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hero',
    maintainer_email='hero@todo.todo',
    description='Blue Robotics Lumen LED PWM control via Jetson sysfs',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lumen_node = lumen_led.lumen_node:main',
            'lumen_ctrl = lumen_led.lumen_ctrl:main',
        ],
    },
)
