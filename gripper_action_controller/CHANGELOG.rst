^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package gripper_action_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.18.1 (2020-12-03)
-------------------
* Format package.xml and CMakeLists.txt + clean deps of gripper_action_controller pkg
* Contributors: Mateus Amarante Araújo

0.18.0 (2020-10-11)
-------------------

0.17.0 (2020-05-12)
-------------------

0.16.1 (2020-04-27)
-------------------

0.16.0 (2020-04-16)
-------------------
* Bump CMake version to prevent CMP0048
* Replace header guard with #pragma once
* Contributors: Matt Reynolds

0.15.1 (2020-03-09)
-------------------
* Use nullptr
* add missing pluginlib deps.
* Contributors: Bence Magyar, Matt Reynolds, Sean Yen

0.15.0 (2019-03-26)
-------------------
* Default all controller builds to C++14
* Use range-based for loops wherever possible
* boost::shared_ptr -> std::shared_ptr
* fix install destination for libraries (`#403 <https://github.com/ros-controls/ros_controllers/issues/403>`_)
* Contributors: Bence Magyar, Gennaro Raiola, James Xu

0.14.3 (2019-02-09)
-------------------
* Use a copy of the pointer in update() to avoid crash by cancelCB()
* Contributors: oka

0.14.2 (2018-10-23)
-------------------

0.14.1 (2018-06-26)
-------------------

0.14.0 (2018-04-27)
-------------------
* migrate to new pluginlib headers
* Contributors: Mathias Lüdtke

0.13.2 (2017-12-23)
-------------------

0.13.1 (2017-11-06)
-------------------

0.13.0 (2017-08-10)
-------------------

0.12.3 (2017-04-23)
-------------------

0.12.2 (2017-04-21)
-------------------

0.12.1 (2017-03-08)
-------------------

0.12.0 (2017-02-15)
-------------------
* Fix most catkin lint issues
* Change for format2
* Add Enrique and Bence to maintainers
* urdf::Model typedefs had to be added to a different repo first
* Replace boost::shared_ptr<urdf::XY> with urdf::XYConstSharedPtr when exists
* Contributors: Bence Magyar

0.11.2 (2016-08-16)
-------------------

0.11.1 (2016-05-23)
-------------------

0.11.0 (2016-05-03)
-------------------

0.10.0 (2015-11-20)
-------------------

0.9.2 (2015-05-04)
------------------

0.9.1 (2014-11-03)
------------------

0.9.0 (2014-10-31)
------------------
* Buildsystem fixes
* Contributors: Adolfo Rodriguez Tsouroukdissian, Lukas Bulwahn

0.8.1 (2014-07-11)
------------------

0.8.0 (2014-05-12)
------------------

0.7.2 (2014-04-01)
------------------
* Added missing deps to package.xml
* Contributors: Scott K Logan

0.7.1 (2014-03-31)
------------------

0.7.0 (2014-03-28)
------------------
* gripper_action_controller: New controller for single dof grippers.
* Contributors: Sachin Chitta.
