
// Header!!
std::string ik_tip_frame_;

// Frame which position should be controlled
std::string endeffector_frame_;

tf2::Transform ik_tip_to_control_frame_tf_;
tf2::Transform control_frame_to_ik_tip_tf_;

// CPP!!
controller_interface::return_type AdmittanceRule::reset()
{
  // Initialize ik_tip and tool_frame transformations - those are fixed transformations
  tf2::Stamped<tf2::Transform> tf2_transform;
  try {
    auto transform = tf_buffer_->lookupTransform(ik_tip_frame_, control_frame_, tf2::TimePointZero);
    tf2::fromMsg(transform, tf2_transform);
    ik_tip_to_control_frame_tf_ = tf2_transform;
    control_frame_to_ik_tip_tf_ = tf2_transform.inverse();
  } catch (const tf2::TransformException & e) {
    RCLCPP_ERROR_SKIPFIRST_THROTTLE(
      rclcpp::get_logger("AdmittanceRule"), *clock_, 5000, "%s", e.what());
    return controller_interface::return_type::ERROR;
  }
}


controller_interface::return_type AdmittanceRule::get_controller_state(
  control_msgs::msg::AdmittanceControllerState & state_message)
{
  // FIXME(destogl): Something is wrong with this transformation - check frames...
  try {
    auto transform = tf_buffer_->lookupTransform(endeffector_frame_, control_frame_, tf2::TimePointZero);
    tf2::doTransform(measured_wrench_ik_base_frame_, measured_wrench_endeffector_frame_, transform);
  } catch (const tf2::TransformException & e) {
    RCLCPP_ERROR_SKIPFIRST_THROTTLE(
      rclcpp::get_logger("AdmittanceRule"), *clock_, 5000, "%s", e.what());
  }
  state_message.measured_wrench_endeffector_frame = measured_wrench_endeffector_frame_;
}
