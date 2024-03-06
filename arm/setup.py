from setuptools import find_packages, setup

package_name = 'arm'

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
    maintainer_email='you@example.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_conf = arm.arm_conf:main',
            'arm_detect = arm.arm_detect:main',
            'arm_detect_move = arm.arm_detect_move:main',
            'arm_listen_control = arm.arm_listen_control:main',
        ],
    },
)
