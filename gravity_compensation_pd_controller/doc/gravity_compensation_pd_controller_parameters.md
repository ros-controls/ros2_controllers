# Gravity Compensation Pd Controller Parameters

Default Config
```yaml
gravity_compensation_pd_controller:
  ros__parameters:
    command_interfaces_names_override:
      effort: '{}'
    compensate_gravity: true
    d_gains: '{}'
    dynamics_solver:
      dynamics_solver_plugin: ''
      root: ''
      tip: ''
    joint_space_reference_controller: ''
    joints: '{}'
    p_gains: '{}'
    robot_name: ''
    state_interfaces_names_override:
      position: '{}'
      velocity: '{}'

```

## joints

Specifies which joints will be used by the controller.

* Type: `string_array`
* Default Value: {}

## robot_name

Name of the robot to control.

* Type: `string`
* Default Value: ""
* Read only: True

## joint_space_reference_controller

Name of the joint space reference controller.

* Type: `string`
* Default Value: ""
* Read only: True

## state_interfaces_names_override.position

If this parameter is set, it will override the default names for the position interface of the state interfaces.

* Type: `string_array`
* Default Value: {}
* Read only: True

## state_interfaces_names_override.velocity

If this parameter is set, it will override the default names for the velocity interface of the state interfaces.

* Type: `string_array`
* Default Value: {}
* Read only: True

## command_interfaces_names_override.effort

If this parameter is set, it will override the default names for the effort interface of the command interfaces.

* Type: `string_array`
* Default Value: {}
* Read only: True

## p_gains

* Type: `double_array`
* Default Value: {}

## d_gains

* Type: `double_array`
* Default Value: {}

## compensate_gravity

If true, the controller will compensate for gravity.

* Type: `bool`
* Default Value: true

## dynamics_solver.dynamics_solver_plugin

Name of the dynamics solver to use.

* Type: `string`
* Read only: True

*Constraints:*
 - parameter is not empty

*Additional Constraints:*



## dynamics_solver.root

Specifies the base link of the robot description used by the inverse dynamics solver.

* Type: `string`
* Read only: True

*Constraints:*
 - parameter is not empty

*Additional Constraints:*



## dynamics_solver.tip

Specifies the end effector link of the robot description used by the inverse dynamics solver.

* Type: `string`
* Read only: True

*Constraints:*
 - parameter is not empty

*Additional Constraints:*
