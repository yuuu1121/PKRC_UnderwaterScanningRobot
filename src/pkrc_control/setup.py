from setuptools import find_packages, setup

package_name = 'pkrc_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 웹 GUI 정적 페이지 — gui_server 가 share 에서 읽는다
        ('share/' + package_name, ['resource/gui.html']),
        ('share/' + package_name + '/launch', ['launch/gui.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hero',
    maintainer_email='hero@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             # live_tuning 은 라이브러리 모듈(main 없음) — entry point 아님
             'gui_server = pkrc_control.gui_server:main',
             'keyboard_control_robust_original = pkrc_control.keyboard_control_robust_original:main',
             'keyboard_control_teleop = pkrc_control.keyboard_control_teleop:main',
             'keyboard_control_wall_align = pkrc_control.keyboard_control_wall_align:main',
             'teleop_logger = pkrc_control.teleop_logger:main',
             'thruster_test = pkrc_control.thruster_test:main',
             'wall_following_logger = pkrc_control.wall_following_logger:main',
        ],
    },
)
