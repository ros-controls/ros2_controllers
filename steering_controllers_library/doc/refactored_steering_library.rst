.. _proposal_refactored_steering_library:

Proposal: A Refactored, Plugin-Based Steering Controller Library
================================================================

Motivation
----------

The current ``steering_controllers_library`` has a monolithic design that tightly couples the core controller logic with specific kinematic implementations (Bicycle, Tricycle, Ackermann). This architecture presents several challenges:

1.  **Lack of Extensibility:** Adding support for new, more complex kinematic models, such as the six-wheel, four-wheel-steering rover discussed in the six_wheel_steering_controllers.rst, requires modifying the core library itself. This is not a scalable solution.
2.  **Configuration Complexity:** The parameter interface is ambiguous and not easily adaptable to different vehicle types.
3.  **High Maintenance Burden:** The central library becomes increasingly complex with each new model, making it difficult to maintain and debug.

This document proposes a new architecture that addresses these issues by creating a truly **kinematics-agnostic** steering controller.

Proposed Architecture: A New, Additive Implementation
------------------------------------------------------

The proposal is to introduce a new, refactored controller, ``generic_steering_controller``, to exist **alongside** the current library. The existing ``steering_controllers_library`` and its controllers will remain untouched, ensuring zero disruption for current users.

The new controller will be built on ``pluginlib``, separating the generic control logic from the specific mathematical models.

Core Components
---------------

**1. The GenericSteeringController**

This will be the main controller class, inheriting from ``controller_interface::ChainableControllerInterface``. Its responsibilities will be limited to:

* Interfacing with the ``ros2_control`` framework and hardware interfaces.
* Managing ROS 2 parameters, subscribers, and publishers.
* Loading the appropriate kinematic model plugin at runtime based on a YAML parameter.
* Calling the plugin's methods within the real-time update loop.

The controller itself will have **zero knowledge** of any specific robot kinematics.

**2. The KinematicModelBase Interface**

A new abstract base class, ``KinematicModelBase``, will be created to define the "contract" that all kinematic plugins must adhere to. This ensures a standard interface between the generic controller and any kinematic model.
.. note::
   To enforce a clean public API and proper separation of concerns, this base class will be defined in its own header file (e.g., ``kinematic_model_base.hpp``).



.. code-block:: cpp

   // A simplified view of the proposed interface
   namespace generic_steering_controller
   {
   class KinematicModelBase
   {
   public:
     virtual ~KinematicModelBase() = default;

     // Called once to allow the plugin to configure itself
     virtual void configure(
       const rclcpp::Node::SharedPtr & node,
       const std::vector<std::string> & traction_joints,
       const std::vector<std::string> & steering_joints) = 0;

     // Called in the real-time loop to calculate commands
     virtual std::pair<std::vector<double>, std::vector<double>>
       get_commands(double linear_vel, double angular_vel) = 0;

     // Called in the real-time loop to update odometry
     virtual void update_odometry(const rclcpp::Duration & period) = 0;
   };
   }

Configuration Example
---------------------

The configuration will be clean and two-tiered. The main controller reads its parameters, and the loaded plugin reads its own.

.. code-block:: yaml

   generic_steering_controller:
     ros__parameters:
       # 1. Controller loads this plugin by name
       kinematics_plugin_name: "six_wheel_kinematics/SixWheelKinematics"

       # Controller gets the joint lists
       traction_joints: ["j1", "j2", "j3", "j4", "j5", "j6"]
       steering_joints: ["s1", "s2", "s3", "s4"]

   # 2. The loaded plugin reads its own specific parameters
   six_wheel_kinematics:
     ros__parameters:
       d1: 0.4
       d2: 0.5
       d3: 0.5
       d4: 0.45
       wheel_radius: 0.15

