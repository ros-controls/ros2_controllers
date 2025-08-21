#include "mock_kinematic_plugin.hpp"
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(mock_kinematic_plugin::MockKinematicModel,
  kinematic_model::KinematicModelBase)
