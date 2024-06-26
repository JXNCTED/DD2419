from setuptools import find_packages, setup

package_name = 'controller'

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
            'cartesian_controller = controller.cartesian_controller:main',
            'arm_controller = controller.arm_controller:main',
            'pursuit_action_server = controller.pursuit_action_server:main',
            'finetune_object_action_server = controller.finetune_object_action_server:main',
            'approach_action_server = controller.approach_action_server:main',
            'dummy_explorer_action_server = controller.dummy_explorer_action_server:main',
            'navigator = controller.navigator:main',
        ],
    },
)
