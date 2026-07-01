from glob import glob
import os

from setuptools import find_packages, setup


package_name = "road_seg_node"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (f"share/{package_name}", ["README.md"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="etri",
    maintainer_email="lee.dk@etri.re.kr",
    description="ROS2 node for realtime road semantic segmentation with Ultralytics YOLO.",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "road_seg_node = road_seg_node.road_seg_node:main",
        ],
    },
)
