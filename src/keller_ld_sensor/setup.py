from setuptools import find_packages, setup

package_name = 'keller_ld_sensor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hero',
    maintainer_email='hero@todo.todo',
    description='ROS 2 driver for Keller LD I2C pressure sensor.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keller_ld_node = keller_ld_sensor.keller_ld_node:main',
            'keller_ld_example = keller_ld_sensor.example:main',
        ],
    },
)
