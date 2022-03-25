# ros2_controllers

Commonly used and generalized controllers for ros2-control framework that are ready to use with many robots, MoveIt2 and Navigation2.

## Build status

ROS2 Distro | Branch | Build status | Documentation | Released packages
:---------: | :----: | :----------: | :-----------: | :---------------:
**Rolling** | [`master`](https://github.com/ros-controls/ros2_controllers/tree/master) | [![Rolling Binary Build](https://github.com/ros-controls/ros2_controllers/actions/workflows/rolling-binary-build.yml/badge.svg?branch=master)](https://github.com/ros-controls/ros2_controllers/actions/workflows/rolling-binary-build.yml?branch=master) <br /> [![Rolling Semi-Binary Build](https://github.com/ros-controls/ros2_controllers/actions/workflows/rolling-semi-binary-build.yml/badge.svg?branch=master)](https://github.com/ros-controls/ros2_controllers/actions/workflows/rolling-semi-binary-build.yml?branch=master) <br /> [![Rolling Source Build](https://github.com/ros-controls/ros2_controllers/actions/workflows/rolling-source-build.yml/badge.svg?branch=master)](https://github.com/ros-controls/ros2_controllers/actions/workflows/rolling-source-build.yml?branch=master) | [Documentation](https://control.ros.org) <br /> [API Reference](https://control.ros.org/rolling/api/) | [ros2_controllers](https://index.ros.org/p/ros2_controllers/#rolling)
**Rolling - last Focal** | [`master`](https://github.com/ros-controls/ros2_controllers/tree/master) | [![Rolling Binary Build](https://github.com/ros-controls/ros2_controllers/actions/workflows/rolling-binary-build-last-focal.yml/badge.svg?branch=master)](https://github.com/ros-controls/ros2_controllers/actions/workflows/rolling-binary-build-last-focal.yml?branch=master) <br /> [![Rolling Semi-Binary Build](https://github.com/ros-controls/ros2_controllers/actions/workflows/rolling-semi-binary-build-last-focal.yml/badge.svg?branch=master)](https://github.com/ros-controls/ros2_controllers/actions/workflows/rolling-semi-binary-build-last-focal.yml?branch=master) | [Documentation](https://control.ros.org) <br /> [API Reference](https://control.ros.org/rolling/api/) | [ros2_controllers](https://index.ros.org/p/ros2_controllers/#rolling)
**Galactic** | [`galactic`](https://github.com/ros-controls/ros2_controllers/tree/galactic) | [![Galactic Binary Build](https://github.com/ros-controls/ros2_controllers/actions/workflows/galactic-binary-build.yml/badge.svg?branch=galactic)](https://github.com/ros-controls/ros2_controllers/actions/workflows/galactic-binary-build.yml?branch=galactic) <br /> [![Galactic Semi-Binary Build](https://github.com/ros-controls/ros2_controllers/actions/workflows/galactic-semi-binary-build.yml/badge.svg?branch=galactic)](https://github.com/ros-controls/ros2_controllers/actions/workflows/galactic-semi-binary-build.yml?branch=galactic) <br /> [![Galactic Source Build](https://github.com/ros-controls/ros2_controllers/actions/workflows/galactic-source-build.yml/badge.svg?branch=galactic)](https://github.com/ros-controls/ros2_controllers/actions/workflows/galactic-source-build.yml?branch=galactic) | [Documentation](https://control.ros.org/galactic/) <br /> [API Reference](https://control.ros.org/galactic/api/) | [ros2_controllers](https://index.ros.org/p/ros2_controllers/#galactic)
**Foxy** | [`foxy`](https://github.com/ros-controls/ros2_controllers/tree/foxy) | [![Foxy Binary Build](https://github.com/ros-controls/ros2_controllers/actions/workflows/foxy-binary-build.yml/badge.svg?branch=foxy)](https://github.com/ros-controls/ros2_controllers/actions/workflows/foxy-binary-build.yml?branch=foxy) <br /> [![Foxy Semi-Binary Build](https://github.com/ros-controls/ros2_controllers/actions/workflows/foxy-semi-binary-build.yml/badge.svg?branch=foxy)](https://github.com/ros-controls/ros2_controllers/actions/workflows/foxy-semi-binary-build.yml?branch=foxy) <br /> [![Foxy Source Build](https://github.com/ros-controls/ros2_controllers/actions/workflows/foxy-source-build.yml/badge.svg?branch=foxy)](https://github.com/ros-controls/ros2_controllers/actions/workflows/foxy-source-build.yml?branch=foxy) | [Documentation](https://control.ros.org/foxy/) <br /> [API Reference](https://control.ros.org/foxy/api/) | [ros2_controllers](https://index.ros.org/p/ros2_controllers/#foxy)


### Explanation of different build types

**NOTE**: There are three build stages checking current and future compatibility of the package.

1. Binary builds - against released packages (main and testing) in ROS distributions. Shows that direct local build is possible.

   Uses repos file: `src/$NAME$/$NAME$-not-released.<ros-distro>.repos`

1. Semi-binary builds - against released core ROS packages (main and testing), but the immediate dependencies are pulled from source.
   Shows that local build with dependencies is possible and if fails there we can expect that after the next package sync we will not be able to build.

   Uses repos file: `src/$NAME$/$NAME$.repos`

1. Source build - also core ROS packages are build from source. It shows potential issues in the mid future.


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
