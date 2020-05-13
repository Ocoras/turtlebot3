import glob
import os

from setuptools import find_packages
from setuptools import setup

package_name = "turtlebot3_path_follower"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(),
    data_files=[
        #     ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        #     # To be added
        #     # ('share/' + package_name + '/launch', glob.glob(os.path.join('launch', 'turtlebot3_interactive_marker.launch.py'))),
        #     # ('share/' + package_name + '/launch', glob.glob(os.path.join('launch', 'turtlebot3_obstacle_detection.launch.py'))),
        #     # ('share/' + package_name + '/rviz', glob.glob(os.path.join('rviz', 'turtlebot3_interactive_marker.rviz'))),
    ],
    install_requires=["setuptools", "launch"],
    zip_safe=True,
    author="David Swarbrick",
    author_email="david.swarbrick9@gmail.com",
    maintainer="David Swarbrick",
    maintainer_email="david.swarbrick9@gmail.com",
    keywords=["ROS", "ROS2", "turtlebot", "rclpy", "path", "follow"],
    # classifiers=[
    #     "Intended Audience :: Developers",
    #     "License :: OSI Approved :: Apache Software License",
    #     "Programming Language :: Python",
    #     "Topic :: Software Development",
    # ],
    description=("TurtleBot3 Path Following Package implemented in Python 3"),
    # license="Apache License, Version 2.0",
    entry_points={"console_scripts": ["follow_path = main:main"]},
)
