import os
from glob import glob

from setuptools import find_packages, setup

package_name = "coug_perception"

setup(
    name=package_name,
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    zip_safe=True,
    extras_require={
        "test": [
            "pytest",
            "pytest-cov",
        ],
    },
    entry_points={
        "console_scripts": [
            "box_overlay = coug_perception.box_overlay_node:main",
            "detection_fusion = coug_perception.detection_fusion_node:main",
            "landmark_tracker = coug_perception.landmark_tracker_node:main",
        ],
    },
)
