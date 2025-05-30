// chained_filter.cpp (migrated from ROSCon 2024 workshop)

#include "chained_filter_controller/chained_filter.hpp"

#include <rclcpp/version.h>
#include <limits>
#include <pluginlib/class_list_macros.hpp>

using namespace chained_filter;

namespace chained_filter_controller
{

controller_interface::CallbackReturn ChainedFilter::on_init()
{
  try
  {
    param_listener_ = std::make_shared<chained_filter::ParamListener>(get_node());
    params_ = param_listener_->get_params();
    filter_ = std::make_unique<filters::FilterChain<double>>("double");
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration ChainedFilter::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration ChainedFilter::state_interface_configuration() const
{
  return {
    controller_interface::interface_configuration_type::INDIVIDUAL, {params_.input_interface}};
}

controller_interface::CallbackReturn ChainedFilter::on_configure(const rclcpp_lifecycle::State &)
{
  params_ = param_listener_->get_params();

  if (!filter_->configure(
        "filter_chain", get_node()->get_node_logging_interface(),
        get_node()->get_node_parameters_interface()))
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Failed to configure filter chain. Check the parameters for filters setup.");
    return controller_interface::CallbackReturn::FAILURE;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ChainedFilter::on_activate(const rclcpp_lifecycle::State &)
{
  output_state_value_ = std::numeric_limits<double>::quiet_NaN();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type ChainedFilter::update_and_write_commands(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  const auto sensor_value = state_interfaces_[0].get_value();

  if (!std::isnan(sensor_value))
  {
    filter_->update(sensor_value, output_state_value_);
  }

  return controller_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> ChainedFilter::on_export_state_interfaces()
{
  return {hardware_interface::StateInterface(
    get_node()->get_name(), params_.output_interface, &output_state_value_)};
}

controller_interface::return_type ChainedFilter::update_reference_from_subscribers(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  return controller_interface::return_type::OK;
}

rclcpp::NodeOptions ChainedFilter::define_custom_node_options() const
{
  return rclcpp::NodeOptions()
    .allow_undeclared_parameters(true)
    .automatically_declare_parameters_from_overrides(false);
}

}  // namespace chained_filter_controller

PLUGINLIB_EXPORT_CLASS(
  chained_filter_controller::ChainedFilter, controller_interface::ChainableControllerInterface)
