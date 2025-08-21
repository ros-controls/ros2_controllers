#ifndef GENERIC_STEERING_CONTROLLER__GENERIC_STEERING_CONTROLLER_HPP_
#define GENERIC_STEERING_CONTROLLER__GENERIC_STEERING_CONTROLLER_HPP_

// C++ Standard Library
#include <memory>
#include <string>
#include <vector>
#include <chrono>

// ros2_control and controller_interface
#include "controller_interface/chainable_controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
// pluginlib
#include "pluginlib/class_loader.hpp"


#include "nav_msgs/msg/odometry.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "control_msgs/msg/steering_controller_status.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>


// rclcpp and realtime_tools
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "realtime_tools/realtime_box.hpp"
#include "realtime_tools/realtime_thread_safe_box.hpp"
// Project-specific
#include "generic_steering_controller/kinematic_model_base.hpp"
#include "generic_steering_controller/generic_steering_controller_parameters.hpp"

namespace generic_steering_controller
{
/**
 * @brief A generic steering controller that uses kinematic plugins.
 *
 * This controller subscribes to a command topic and uses a dynamically loaded
 * plugin to calculate wheel and steering commands for various robot kinematics.
 */
class GenericSteeringController : public controller_interface::ChainableControllerInterface
{
public:
  GenericSteeringController();

  virtual void initialize_implementation_parameter_listener();

  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn configure_odometry();

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update_reference_from_subscribers(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  using ControllerTwistReferenceMsg = geometry_msgs::msg::TwistStamped;
  using ControllerStateMsgOdom = nav_msgs::msg::Odometry;
  using ControllerStateMsgTf = tf2_msgs::msg::TFMessage;
  using GenericSteeringControllerStateMsg = control_msgs::msg::SteeringControllerStatus;

  inline std::unordered_map<std::string, double> get_state_interface_map() const
  {
    std::unordered_map<std::string, double> current_map;
    for (const auto & iface : state_interfaces_) {
      current_map[iface.get_name()] = iface.get_value();
    }
    return current_map;
  }

protected:
  controller_interface::CallbackReturn set_interface_numbers(
    size_t nr_state_itfs, size_t nr_cmd_itfs, size_t nr_ref_itfs);


  std::shared_ptr<generic_steering_controller_parameters::ParamListener> param_listener_;
  generic_steering_controller_parameters::Params params_;

  // the RT Box containing the command message
  realtime_tools::RealtimeThreadSafeBox<ControllerTwistReferenceMsg> input_ref_;
  ControllerTwistReferenceMsg current_ref_;
  rclcpp::Duration ref_timeout_ = rclcpp::Duration::from_seconds(0.0);  // 0ms

  // Command subscribers and Controller State publisher
  rclcpp::Subscription<ControllerTwistReferenceMsg>::SharedPtr ref_subscriber_twist_ = nullptr;
  using ControllerStatePublisherOdom = realtime_tools::RealtimePublisher<ControllerStateMsgOdom>;
  using ControllerStatePublisherTf = realtime_tools::RealtimePublisher<ControllerStateMsgTf>;

  rclcpp::Publisher<ControllerStateMsgOdom>::SharedPtr odom_s_publisher_;
  rclcpp::Publisher<ControllerStateMsgTf>::SharedPtr tf_odom_s_publisher_;

  std::unique_ptr<ControllerStatePublisherOdom> rt_odom_state_publisher_;
  std::unique_ptr<ControllerStatePublisherTf> rt_tf_odom_state_publisher_;

  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

  bool on_set_chained_mode(bool chained_mode) override;

  /// Odometry:

  GenericSteeringControllerStateMsg published_state_;

  using ControllerStatePublisher = realtime_tools::RealtimePublisher<GenericSteeringControllerStateMsg>;
  rclcpp::Publisher<GenericSteeringControllerStateMsg>::SharedPtr controller_s_publisher_;
  std::unique_ptr<ControllerStatePublisher> controller_state_publisher_;

  // name constants for state interfaces
  size_t nr_state_itfs_;
  // name constants for command interfaces
  size_t nr_cmd_itfs_;
  // name constants for reference interfaces
  size_t nr_ref_itfs_;

  // last velocity commands for open loop odometry
  double last_linear_velocity_ = 0.0;
  double last_angular_velocity_ = 0.0;

  std::vector<std::string> traction_joints_state_names_;
  std::vector<std::string> steering_joints_state_names_;
  bool open_loop_;

  void reference_callback(const std::shared_ptr<ControllerTwistReferenceMsg> msg);

private:
// callback for topic interface

  pluginlib::ClassLoader<kinematic_model::KinematicModelBase> kinematic_loader;
  std::shared_ptr<kinematic_model::KinematicModelBase> kinematic_model_;
  std::unordered_map<std::string, double> state_map;
  std::string kinematic_plugin_name;
};

}  // namespace generic_steering_controller

#endif  //GENERIC_STEERING_CONTROLLER__GENERIC_STEERING_CONTROLLER_HPP_
