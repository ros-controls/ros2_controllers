# ros2_controllers

[![Licence](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![codecov](https://codecov.io/gh/ros-controls/ros2_controllers/graph/badge.svg?token=KSdY0tsHm6)](https://codecov.io/gh/ros-controls/ros2_controllers)

Commonly used and generalized controllers for ros2-control framework that are ready to use with many robots, MoveIt2 and Nav2.

## Build status

ROS2 Distro | Branch | Build status | Documentation | Released packages
:---------: | :----: | :----------: | :-----------: | :---------------:
**Rolling** | [`master`](https://github.com/ros-controls/ros2_controllers/tree/master) | [![Rolling Binary Build](https://github.com/ros-controls/ros2_controllers/actions/workflows/rolling-binary-build.yml/badge.svg?branch=master)](https://github.com/ros-controls/ros2_controllers/actions/workflows/rolling-binary-build.yml?branch=master) <br /> [![Rolling Semi-Binary Build](https://github.com/ros-controls/ros2_controllers/actions/workflows/rolling-semi-binary-build.yml/badge.svg?branch=master)](https://github.com/ros-controls/ros2_controllers/actions/workflows/rolling-semi-binary-build.yml?branch=master) | [control.ros.org](https://control.ros.org/master/doc/ros2_controllers/doc/controllers_index.html) | [ros2_controllers](https://index.ros.org/p/ros2_controllers/#rolling)
**Jazzy** | [`master`](https://github.com/ros-controls/ros2_controllers/tree/master) | [![Jazzy Binary Build](https://github.com/ros-controls/ros2_controllers/actions/workflows/jazzy-binary-build.yml/badge.svg?branch=master)](https://github.com/ros-controls/ros2_controllers/actions/workflows/jazzy-binary-build.yml?branch=master) <br /> [![Jazzy Semi-Binary Build](https://github.com/ros-controls/ros2_controllers/actions/workflows/jazzy-semi-binary-build.yml/badge.svg?branch=master)](https://github.com/ros-controls/ros2_controllers/actions/workflows/jazzy-semi-binary-build.yml?branch=master) | [control.ros.org](https://control.ros.org/jazzy/doc/ros2_controllers/doc/controllers_index.html) | [ros2_controllers](https://index.ros.org/p/ros2_controllers/#jazzy)
**Iron** | [`iron`](https://github.com/ros-controls/ros2_controllers/tree/iron) | [![Iron Binary Build](https://github.com/ros-controls/ros2_controllers/actions/workflows/iron-binary-build.yml/badge.svg?branch=master)](https://github.com/ros-controls/ros2_controllers/actions/workflows/iron-binary-build.yml?branch=master) <br /> [![Iron Semi-Binary Build](https://github.com/ros-controls/ros2_controllers/actions/workflows/iron-semi-binary-build.yml/badge.svg?branch=master)](https://github.com/ros-controls/ros2_controllers/actions/workflows/iron-semi-binary-build.yml?branch=master) | [control.ros.org](https://control.ros.org/iron/doc/ros2_controllers/doc/controllers_index.html) | [ros2_controllers](https://index.ros.org/p/ros2_controllers/#iron)
**Humble** | [`humble`](https://github.com/ros-controls/ros2_controllers/tree/humble) | [![Humble Binary Build](https://github.com/ros-controls/ros2_controllers/actions/workflows/humble-binary-build.yml/badge.svg?branch=master)](https://github.com/ros-controls/ros2_controllers/actions/workflows/humble-binary-build.yml?branch=master) <br /> [![Humble Semi-Binary Build](https://github.com/ros-controls/ros2_controllers/actions/workflows/humble-semi-binary-build.yml/badge.svg?branch=master)](https://github.com/ros-controls/ros2_controllers/actions/workflows/humble-semi-binary-build.yml?branch=master) | [control.ros.org](https://control.ros.org/humble/doc/ros2_controllers/doc/controllers_index.html) | [ros2_controllers](https://index.ros.org/p/ros2_controllers/#humble)

### Explanation of different build types

**NOTE**: There are three build stages checking current and future compatibility of the package.

1. Binary builds - against released packages (main and testing) in ROS distributions. Shows that direct local build is possible.

   Uses repos file: `src/$NAME$/$NAME$-not-released.<ros-distro>.repos`

1. Semi-binary builds - against released core ROS packages (main and testing), but the immediate dependencies are pulled from source.
   Shows that local build with dependencies is possible and if fails there we can expect that after the next package sync we will not be able to build.

   Uses repos file: `src/$NAME$/$NAME$.repos`


## Acknowledgements

<!--
    ROSIN acknowledgement from the ROSIN press kit
    @ https://github.com/rosin-project/press_kit
-->

<a href="http://rosin-project.eu">
  <img src="http://rosin-project.eu/wp-content/uploads/rosin_ack_logo_wide.png"
       alt="rosin_logo" height="60" >
</a>

Supported by ROSIN - ROS-Industrial Quality-Assured Robot Software Components.
More information: <a href="http://rosin-project.eu">rosin-project.eu</a>

<img src="http://rosin-project.eu/wp-content/uploads/rosin_eu_flag.jpg"
     alt="eu_flag" height="45" align="left" >

This project has received funding from the European Union’s Horizon 2020
research and innovation programme under grant agreement no. 732287.
