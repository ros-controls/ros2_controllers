admittance_controller
==========================================

The admittance controller operates in one of two modes: chained and non-chained.
In the non-chained mode, the controller listens to joint commands on the `~/joint_commands` topic to get a reference. In chained mode, the admittance controller receives its reference from another controller. In both modes, the controller applies the calculated admittance values to the received reference and write the modified reference to its command interfaces.

The controller requires an external kinematics plugin to function. The [kinematics_interface](https://github.com/ros-controls/kinematics_interface) repository provides an interface and an implementation that the admittance controller can use.

## Parameters:
admittance.damping_ratio
```
Type: double array
Description: specifies damping ratio values for x, y, z, rx, ry, and rz used in the admittance calculation. The values are calculated as damping can be used instead: zeta = D / (2 * sqrt( M * S ))
```

admittance.mass
```
Type: double array
Description: specifies mass values for x, y, z, rx, ry, and rz used in the admittance calculation
```

admittance.selected_axes
```
Type: double array
Description: specifies if the axes x, y, z, rx, ry, and rz are enabled
```

admittance.stiffness
```
Type: double array
Description: specifies stiffness values for x, y, z, rx, ry, and rz used in the admittance calculation
```

chainable_command_interfaces
```
Type: string array
Description: specifies which chainable interfaces to claim
```

command_interfaces
```
Type: string array
Description: specifies which command interfaces to claim
```

control.frame.id
```
Type: string
Description: control frame used for admittance control
```

enable_parameter_update_without_reactivation
```
Type: boolean
Description: if enabled, parameters will be dynamically updated in the control loop
```

fixed_world_frame.frame.id
```
Type: string
Description: world frame, gravity points down (neg. Z) in this frame
```

ft_sensor.filter_coefficient
```
Type: double
Description: specifies the coefficient for the sensor's exponential filter
```

ft_sensor.frame.id
```
Type: string
Description: frame of the force torque senso
```

ft_sensor.name
```
Type: string
Description: name of the force torque sensor in the robot description
```

gravity_compensation.CoG.force
```
Type: double
Description: weight of the end effector, e.g mass * 9.81
```

gravity_compensation.CoG.pos
```
Type: double array
Description: position of the center of gravity (CoG) in its frame
```

gravity_compensation.frame.id
```
Type: string
Description: rame which center of gravity (CoG) is defined in
```

joints
```
Type: string array
Description: specifies which joints will be used by the controller
```

kinematics.alpha
```
Type: double
Description: specifies the damping coefficient for the Jacobian pseudo inverse
```

kinematics.base
```
Type: string
Description: specifies the base link of the robot description used by the kinematics plugin
```

kinematics.plugin_name
```
Type: string
Description: specifies which kinematics plugin to load
```

kinematics.plugin_package
```
Type: string
Description: specifies the package to load the kinematics plugin from
```

kinematics.tip
```
Type: string
Description: specifies the end effector link of the robot description used by the kinematics plugin
```

robot_description
```
Type: string
Description: Contains robot description in URDF format. The description is used to perform forward and inverse kinematics.
```

state_interfaces
```
Type: string array
Description: specifies which state interfaces to claim
```
