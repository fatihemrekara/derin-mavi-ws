import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'line_perception'

setup(
    name=package_name,
    version='1.0.0',
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
    maintainer='AUV Team',
    maintainer_email='auv@example.com',
    description='YOLO11-seg ve OpenCV kullanarak su altı hat tespiti',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_segmentation_node = line_perception.yolo_segmentation_node:main',
            'line_extraction_node = line_perception.line_extraction_node:main',
        ],
    },
)
