:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/doc/migration.rst

<<<<<<< HEAD
Migration Guides: Galactic to Humble
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
This list summarizes important changes between Galactic (previous) and Humble (current) releases, where changes to user code might be necessary.

.. note::

  This list was created in July 2024, earlier changes are not included.

joint_trajectory_controller
*****************************
  * Tolerances sent with the action goal were not used before, but are now processed and used for the upcoming action. (`#716 <https://github.com/ros-controls/ros2_controllers/pull/716>`_). Adaptions to the action goal might be necessary.
=======
Migration Guides: Jazzy to Kilted
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This list summarizes important changes between Jazzy (previous) and Kilted (current) releases, where changes to user code might be necessary.
>>>>>>> b75fe9e (Add Kilted Kaiju (#1646))
