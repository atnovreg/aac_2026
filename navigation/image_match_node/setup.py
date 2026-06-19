"""Package definition for image_match_node."""

from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'image_match_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name), ['README.md']),
    ],
    install_requires=[
        'setuptools',
        'safetensors>=0.4.3',
        'transformers>=4.56,<5',
    ],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='etri',
    maintainer_email='lee.dk@etri.re.kr',
    description='OpenCV and neural map-based visual localization for ROS 2',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'efficientloftr_localization_node = '
            'efficientloftr_node.image_match_node:main',
            'prepare_efficientloftr_cache = efficientloftr_node.prepare_cache:main',
            'minima_localization_node = minima_node.image_match_node:main',
            'opencv_localization_node = opencv_node.image_match_node:main',
            'map_odom_tf_node = localization_tf_node.map_odom_tf_node:main',
        ],
    },
)
