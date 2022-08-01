#!/usr/bin/env python

from setuptools import setup
from glob import glob

package_name = "rqt_joint_trajectory_controller"

setup(
    name=package_name,
    version="2.10.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/resource", glob("resource/*.*")),
        ("share/" + package_name, ["plugin.xml"]),

    ],
    install_requires=["setuptools"],
    zip_safe=True,
    author="Noel Jimenez Garcia",
    author_email="noel.jimenez@pal-robotics.com",
    maintainer="Bence Magyar",
    maintainer_email="bence.magyar.robotics@gmail.com",
    license="Apache License, Version 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "rqt_joint_trajectory_controller = \
                rqt_joint_trajectory_controller.rqt_joint_trajectory_controller:main",
        ],
    },
)
