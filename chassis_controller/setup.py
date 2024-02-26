from setuptools import find_packages, setup

package_name = 'chassis_controller'

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
    maintainer='group7',
    maintainer_email='group7@todo.todo',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joy_controller = chassis_controller.joy_controller:main',
            'obj_detect_move = chassis_controller.obj_detect_move:main',
            'aruco_detect_move = chassis_controller.aruco_detect_move:main',
            'nav_goal_move = chassis_controller.nav_goal_move:main',
            'arm_conf = chassis_controller.arm_conf:main',
            'arm_detect = chassis_controller.arm_detect:main',
            'arm_listen_control = chassis_controller.arm_listen_control:main'
        ],
    },
)
