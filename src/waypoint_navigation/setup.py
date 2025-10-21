import os
from glob import glob

from setuptools import setup

package_name = "waypoint_navigation"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "map"), glob("map/*")),
    ],
    install_requires=[
        "setuptools",
        "rclpy",
        "nav2_msgs",
        "geometry_msgs",
        "pyyaml",
        "PyQt5",
    ],
    zip_safe=True,
    maintainer="Manan Dhinora",
    maintainer_email="dhinoramanan@gmail.com",
    description="ROS 2 Nav2 Waypoint Commander with PyQt GUI",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "waypoint_manager_node = waypoint_navigation.waypoint_manager_node:main",
            "waypoint_gui_node = waypoint_navigation.waypoint_gui_node:main_gui",
            "auto_initial_pose_node = waypoint_navigation.auto_initial_pose_node:main",
        ],
    },
)
