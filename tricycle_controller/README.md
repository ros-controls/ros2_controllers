# Controllers


### controllers are derived from the controller_interface in ros2_control and exported as a plugins.

### update method is called in the control-loop to access current hardware state and write the hardware's command interfaces.

### controllers defined in a YAML file with the configuration of the controller manager and the controllers

### we define the tricycle-like controller which is going to defined with two joints: steering joint and wheel front joint. This controller is responsible to get the speed and steering angle and set the command interface of both joints, via the hardware interface defined.


## Hardware Interfaces defined For tricycle controller:
- ### Front Wheel Hardware Interface (Amr Wheel)
    - define three hardware interfaces:
        - velocity state
        - velocity command
    - define get_velocity function to get the velocity state
    - define set_velocity function to set the velocity command
- ### Steering Joint Hardware Interface (Amr Steer)
   - define three hardware interfaces:
        - position state
        - position command
    - define get_position function to get the position state
    - define set_position function to set the position command

## Main Functions

- ### command_interface_configuration() & state_interface_configuration()
    define the command and state interfaces for each joint
- ### on_activate()
    - declare any parameters needed
    - initialize the joint hardware interfaces
- ### get_wheel(wheel_joint_name) & get_steer(wheel_joint_name)
    - initialize the hardware interface of the steering_joint and front_wheel_joint in order to be able to read the states and set commands to each of the joints defined:
        - get the states defined for the joint (position/velocity or both)
        - get the command defined for the joint (position/velocity)
        - create a joint instance from one of the two object classes defined: Amr Wheel or Amr Steer.
- ### on_configure()
    - get joint paramaters from defined YAML file of controller.
    - initialize subscribers and publishers

- ### update()
    - get the current state of the joints
    - transform the current states of the joints to the state of the base_link and publish them via odometry message on topic /wheel_odom_node/odom
    - get the last velocity command from the /cmd_vel topic
    - if states from gazebo, convert from radians/second to meters/second
    - compare with current states and apply limits
    - reconvert states to radians/second and set the command interfaces for the velocity/position of both joints



- ### on_cleanup()
    - reset joints

## Subscribers & Publishers
 - odometry publisher for base_link (rear wheels): wheel_odom_node/odom
 - velocity_command subscriber: cmd_vel

## YAML File
```yaml
agv2:
    tricycle_controller_manager:
    ros__parameters:
        update_rate: 50 # Hz

        tricycle_controller:
        type: tricycle_controller/TricycleController

    tricycle_controller:
    ros__parameters:
        wheel_front_joint_name: wheel_front_joint
        steering_joint_name: steering_joint

```
