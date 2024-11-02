#!/usr/bin/env python

# Copyright 2024 Apache License, Version 2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from setuptools import setup
from glob import glob

package_name = "rqt_joint_trajectory_controller"

setup(
    name=package_name,
    version="3.27.0",
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
