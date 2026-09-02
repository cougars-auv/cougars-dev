import os
from glob import glob

from setuptools import find_packages, setup

package_name = "coug_bringup"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="snelsondurrant",
    maintainer_email="snelsond@byu.edu",
    description="System-wide ROS 2 launch configurations.",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
            "pytest-cov",
        ],
    },
    entry_points={
        "console_scripts": [
            "bag_recorder = coug_bringup.bag_recorder_node:main",
        ],
    },
)
