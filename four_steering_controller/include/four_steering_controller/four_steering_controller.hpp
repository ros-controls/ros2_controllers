#ifndef FOUR_STEERING_CONTROLLER__FOUR_STEERING_CONTROLLER_HPP_
#define FOUR_STEERING_CONTROLLER__FOUR_STEERING_CONTROLLER_HPP_

#include <chrono>
#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "four_steering_controller/odometry.hpp"
#include "four_steering_controller/speed_limiter.hpp"
#include "four_steering_controller/visibility_control.h"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "hardware_interface/handle.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_box.h"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "std_srvs/srv/empty.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

#include "four_steering_controller_parameters.hpp"
#include "four_wheel_steering_msgs/msg/four_wheel_steering_stamped.hpp"
#include "four_wheel_steering_msgs/msg/four_wheel_steering.hpp"

namespace four_steering_controller
{
class FourSteeringController : public controller_interface::ControllerInterface
{

public:
  FOUR_STEERING_CONTROLLER_PUBLIC
  FourSteeringController();

  FOUR_STEERING_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  FOUR_STEERING_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  FOUR_STEERING_CONTROLLER_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  FOUR_STEERING_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  FOUR_STEERING_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  FOUR_STEERING_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  FOUR_STEERING_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  FOUR_STEERING_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  FOUR_STEERING_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  FOUR_STEERING_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  template<typename T>
  T clamp(T x, T min, T max)
  {
    return std::min(std::max(min, x), max);
  };

protected:
  // Parte nuova:
  struct SteerHandle
  {
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> feedback_pos;
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> position;
  };
  struct WheelHandle
  {
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> feedback;
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity;
  };
 
  controller_interface::CallbackReturn configure_wheels(
    const std::vector<std::string> & wheel_names,
    std::vector<WheelHandle> & registered_handles);
  
  controller_interface::CallbackReturn configure_steers(
    const std::vector<std::string> & steer_names,
    std::vector<SteerHandle> & registered_handles);
    
  std::vector<WheelHandle> registered_wheel_handles_;
  std::vector<SteerHandle> registered_steer_handles_;

  // Parameters from ROS for diff_drive_controller
  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  Odometry odometry_;

  // Timeout to consider cmd_vel commands old
  std::chrono::milliseconds cmd_vel_timeout_{500};

  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odometry_publisher_ = nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>
    realtime_odometry_publisher_ = nullptr;

  std::shared_ptr<rclcpp::Publisher<tf2_msgs::msg::TFMessage>> odometry_transform_publisher_ =
    nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>
    realtime_odometry_transform_publisher_ = nullptr;

  bool subscriber_is_active_ = false;
  rclcpp::Subscription<four_wheel_steering_msgs::msg::FourWheelSteering>::SharedPtr velocity_command_unstamped_subscriber_ = nullptr;
  rclcpp::Subscription<four_wheel_steering_msgs::msg::FourWheelSteeringStamped>::SharedPtr velocity_command_stamped_subscriber_ = nullptr;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_twist_cmd_stamped_subscriber_ = nullptr;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_twist_cmd_unstamped_subscriber_ = nullptr;
  
  realtime_tools::RealtimeBox<std::shared_ptr<four_wheel_steering_msgs::msg::FourWheelSteeringStamped>> received_velocity_msg_ptr_{nullptr};
  realtime_tools::RealtimeBox<std::shared_ptr<geometry_msgs::msg::TwistStamped>> received_velocity_twist_msg_ptr_{nullptr};


  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_odom_service_;

  std::queue<four_wheel_steering_msgs::msg::FourWheelSteeringStamped> previous_commands_;  // last two commands
  std::queue<geometry_msgs::msg::TwistStamped> previous_twist_commands_;  
  rclcpp::Time previous_update_timestamp_{0};

  //Limiters
  SpeedLimiter limiter_lin_;
  SpeedLimiter limiter_ang_;

  bool is_halted = false;
  bool use_stamped_vel_ = true;
  
  geometry_msgs::msg::TwistStamped last0_cmd_;
  geometry_msgs::msg::TwistStamped last1_cmd_;
  four_wheel_steering_msgs::msg::FourWheelSteeringStamped last0_cmd_4ws_;
  four_wheel_steering_msgs::msg::FourWheelSteeringStamped last1_cmd_4ws_;

  double wheel_vel_cmd[4] = { 0 };
  double steer_cmd[4] = { 0 };
  double ws_read[4] = { 0 };
  double alphas_read[4] = { 0 };
  
  double vel_left_front = 0.0, vel_right_front = 0.0;
  double vel_left_rear = 0.0, vel_right_rear = 0.0;
  double front_left_steering = 0.0, front_right_steering = 0.0;
  double rear_left_steering = 0.0, rear_right_steering = 0.0;
  double wheel_vel = 0.0;
  
  void reset_odometry(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Empty::Request> req,
    std::shared_ptr<std_srvs::srv::Empty::Response> res);
  bool reset();
  void halt();
};
}  // namespace four_steering_controller
#endif  // FOUR_STEERING_CONTROLLER__FOUR_STEERING_CONTROLLER_HPP_
