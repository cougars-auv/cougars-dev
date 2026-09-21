import os
from glob import glob

from setuptools import find_packages, setup

package_name = "coug_gazebo"

setup(
    name=package_name,
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*.sdf.xacro")),
        *[
            (os.path.join("share", package_name, os.path.dirname(path)), [path])
            for path in glob("models/**/*.*", recursive=True)
        ],
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
            "dem_global_costmap = coug_gazebo.dem_global_costmap_node:main",
            "imu_covariance = coug_gazebo.imu_covariance_node:main",
            "mag_covariance = coug_gazebo.mag_covariance_node:main",
            "navsat_covariance = coug_gazebo.navsat_covariance_node:main",
        ],
    },
)
