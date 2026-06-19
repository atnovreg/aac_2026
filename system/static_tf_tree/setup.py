"""Package definition for static_tf_tree."""

from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'static_tf_tree'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml'),
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
        ('share/' + package_name, ['README.md']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='etri',
    maintainer_email='lee.dk@etri.re.kr',
    description='Central owner for AAC fixed TF transforms',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'static_tf_tree_node = '
            'static_tf_tree.static_tf_tree_node:main',
        ],
    },
)
