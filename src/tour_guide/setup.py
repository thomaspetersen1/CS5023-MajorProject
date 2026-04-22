import os
from glob import glob

from setuptools import find_packages, setup

package_name = "tour_guide"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "maps"), glob("maps/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Franco Barbaro and Thomas Petersen",
    maintainer_email="fbarbaro@ou.edu",
    description="Tour Guide for the Robotics Lab at OU",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "route_planner = tour_guide.route_planner:main",
            "tour_executor = tour_guide.tour_executor:main",
            "landmark_publisher = tour_guide.landmark_publisher:main",
        ],
    },
)
