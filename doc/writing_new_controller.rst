.. _writing_new_controllers:

Writing a new controller
========================

In this framework controllers are libraries, dynamically loaded by the controller manager using the `pluginlib <ros.org/wiki/pluginlib>`_ interface.
The following is a step-by-step guide to create source files, basic tests, and compile rules for a new controller.

1. **Preparing package**

   If the package for the controller does not exist, then create it first.
   The package should have ``ament_cmake`` as a build type.
   The easiest way is to search online for the most recent manual.
   A helpful command to support this process is ``ros2 pkg create``.
   Use the ``--help`` flag for more information on how to use it.
   There is also an option to create library source files and compile rules to help you in the following steps.

2. **Preparing source files**

   After creating the package, you should have at least ``CMakeLists.txt`` and ``package.xml`` files in it.
   Create also ``include/<PACKAGE_NAME>/`` and ``src`` folders if they do not already exist.
   In ``include/<PACKAGE_NAME>/`` folder add ``<controller_name>.hpp`` and ``<controller_name>.cpp`` in the ``src`` folder.
   Optionally add ``visibility_control.h`` with the definition of export rules for Windows.
   You can copy this file from an existing controller package and change the name prefix to the ``<PACKAGE_NAME>``.

3. **Adding declarations into header file (.hpp)**

   1. Take care that you use header guards. ROS2-style is using ``#ifndef`` and ``#define`` preprocessor directives. (For more information on this, a search engine is your friend :) ).

   2. include ``"controller_interface/controller_interface.hpp"`` and ``visibility_control.h`` if you are using one.

   3. Define a unique namespace for your controller. This is usually a package name written in ``snake_case``.

   4. Define the class of the controller, extending ``ControllerInterface``, e.g.,
      .. code:: c++
      class ControllerName : public controller_interface::ControllerInterface

   5. Add a constructor without parameters and the following public methods overriding the ``ControllerInterface`` definition: ``init``, ``command_interface_configuration``, ``state_interface_configuration``, ``on_configure``, ``on_activate``, ``on_deactivate``, ``update``.
      For exact definitions check the ``controller_interface/controller_interface.hpp`` header or one of the controllers from `ros2_controllers <https://github.com/ros-controls/ros2_controllers>`_.

   6. (optional) Often, controllers accept lists of joint names and interface names as parameters.
      If so, you can add two protected string vectors to store those values.

4. **Adding definitions into source file (.cpp)**

   1. Include the header file of your controller and add a namespace definition to simplify further development.

   2. (optional) Implement a constructor if needed. There, you could initialize member variables.
      This could also be done in the ``init`` method.

   3. Implement the ``init`` method. The first line usually calls the parent ``init`` method.
      Here is the best place to initialize the variables, reserve memory, and most importantly, declare node parameters used by the controller. If everything works fine return ``controller_interface::return_type::OK`` or ``controller_interface::return_type::ERROR`` otherwise.

   4. Write the ``on_configure`` method. Parameters are usually read here, and everything is prepared so that the controller can be started.

   5. Implement ``command_interface_configuration`` and ``state_interface_configuration`` where required interfaces are defined.
      There are three options of the interface configuration ``ALL``, ``INDIVIDUAL``, and ``NONE`` defined in ``controller_interface/controller_interface.hpp"``.
      ``ALL`` and ``NONE`` option will ask for access to all available interfaces or none of them. The ``INDIVIDUAL`` configuration needs a detailed list of required interface names. Those are usually provided as parameters.
      The full interface names have structure ``<joint_name>/<interface_type>``.

   6. Implement the ``on_activate`` method with checking, and potentially sorting, the interfaces and assigning members' initial values.
      This method is part of the real-time loop, therefore avoid any reservation of memory and, in general, keep it as short as possible.


   7. Implement the ``on_deactivate`` method, which does the opposite of ``on_activate``.
      In many cases, this method is empty.
      This method should also be real-time safe as much as possible.

   8. Implement the ``update`` method as the main entry point. The method should be implemented with `real-time <https://en.wikipedia.org/wiki/Real-time_computing>`_ constraints in mind.
      When this method is called, the state interfaces have the most recent values from the hardware, and new commands for the hardware should be written into command interfaces.

   9. IMPORTANT: At the end of your file after the namespace is closed, add the ``PLUGINLIB_EXPORT_CLASS`` macro.
      For this you will need to include the ``"pluginlib/class_list_macros.hpp"`` header.
      As first parameters you should provide exact controller class, e.g., ``<controller_name_namespace>::<ControllerName>``, and as second the base class, i.e., ``controller_interface::ControllerInterface``.

5. **Writing export definition for pluginlib**

   1. Create the ``<controller_name>.xml`` file in the package and add a definition of the library and controller's class which has to be visible for the pluginlib.
      The easiest way to do that is to check other controllers in the `ros2_controllers <https://github.com/ros-controls/ros2_controllers>`_ package.

   2. Usually, the plugin name is defined by the package (namespace) and the class name, e.g.,
      ``<controller_name_package>/<ControllerName>``.
      This name defines the controller's type when the controller manager searches for it.
      The other two files have to correspond to the definition done in macro at the bottom of the ``<controller_name>.cpp`` file.

6. **Writing simple test to check if the controller can be found and loaded**

   1. Create the folder ``test`` in your package, if it does not exist already, and add a file named ``test_load_<controller_name>.cpp``.

   2. You can safely copy the file's content for any controller defined in the `ros2_controllers <https://github.com/ros-controls/ros2_controllers>`_ package.

   3. Change the name of the copied test and in the last line, where controller type is specified put the name defined in ``<controller_name>.xml`` file, e.g., ``<controller_name_package>/<ControllerName>``.

7. **Add compile directives into ``CMakeLists.txt`` file**

   1. Under the line ``find_package(ament_cmake REQUIRED)`` add further dependencies.
      Those are at least: ``controller_interface``, ``hardware_interface``, ``pluginlib``, ``rclcpp`` and ``rclcpp_lifecycle``.

   2. Add a compile directive for a shared library providing the ``<controller_name>.cpp`` file as the source.

   3. Add targeted include directories for the library. This is usually only ``include``.

   4. Add ament dependencies needed by the library. You should add at least those listed under 1.

   5. Export for pluginlib description file using the following command:
      .. code:: cmake

         pluginlib_export_plugin_description_file(controller_interface <controller_name>.xml)

   6. Add install directives for targets and include directory.

   7. In the test section add the following dependencies: ``ament_cmake_gmock``, ``controller_manager``, ``hardware_interface``, ``ros2_control_test_assets``.

   8. Add compile definitions for the tests using the ``ament_add_gmock`` directive.
      For details, see how it is done for controllers in the `ros2_controllers <https://github.com/ros-controls/ros2_controllers>`_ package.

   9. (optional) Add your controller`s library into ``ament_export_libraries`` before ``ament_package()``.

8. **Add dependencies into ``package.xml`` file**

   1. Add at least the following packages into ``<depend>`` tag: ``controller_interface``, ``hardware_interface``, ``pluginlib``, ``rclcpp`` and ``rclcpp_lifecycle``.

   2. Add at least the following package into ``<test_depend>`` tag: ``ament_add_gmock``, ``controller_manager``, ``hardware_interface``, and ``ros2_control_test_assets``.

9. **Compiling and testing the controller**

   1. Now everything is ready to compile the controller using the ``colcon build <controller_name_package>`` command.
      Remember to go into the root of your workspace before executing this command.

   2. If compilation was successful, source the ``setup.bash`` file from the install folder and execute ``colcon test <controller_name_package>`` to check if the new controller can be found through ``pluginlib`` library and be loaded by the controller manager.


That's it! Enjoy writing great controllers!


Useful External References
---------------------------

- `Templates and scripts for generating controllers shell <https://stoglrobotics.github.io/ros_team_workspace/use-cases/ros2_control/setup_controller.html>`_
