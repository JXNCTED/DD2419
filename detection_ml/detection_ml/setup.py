from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'detection_ml'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(
            os.path.join('dd2419_detector_baseline', '*.[pt]*'))),
        (os.path.join('share', package_name, 'launch'), glob(
            os.path.join('dd2419_detector_baseline', '*.[py]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ed',
    maintainer_email='33616271+JXNCTED@users.noreply.github.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detection_ml = detection_ml.detection_ml:main',
            'category_eval = detection_ml.category_eval:main'
        ],
    },
)
