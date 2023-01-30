#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "four_steering_controller/four_steering_controller.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace
{
constexpr auto DEFAULT_COMMAND_TOPIC = "~/cmd_vel";
constexpr auto DEFAULT_COMMAND_UNSTAMPED_TOPIC = "~/cmd_vel_unstamped";
constexpr auto DEFAULT_COMMAND_OUT_TOPIC = "~/cmd_vel_out";
constexpr auto DEFAULT_ODOMETRY_TOPIC = "~/odom";
constexpr auto DEFAULT_TRANSFORM_TOPIC = "/tf";
}  // namespace

namespace four_steering_controller
{
using namespace std::chrono_literals;
using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using lifecycle_msgs::msg::State;


FourSteeringController::FourSteeringController() : controller_interface::ControllerInterface() {}

const char * FourSteeringController::feedback_type() const
{
  return params_.position_feedback ? HW_IF_POSITION : HW_IF_VELOCITY;
}

controller_interface::CallbackReturn FourSteeringController::on_init()
{
  try
  {
    // Create the parameter listener and get the parameters
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
    auto log = get_node()->get_logger();
    RCLCPP_WARN(log, "Parameters read correctly.");

  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

// Get joint names from the parameter 
InterfaceConfiguration FourSteeringController::command_interface_configuration() const
{
  std::vector<std::string> conf_names;
  for (const auto & joint_name : params_.wheel_names)
  {
    conf_names.push_back(joint_name + "/" + HW_IF_VELOCITY);
  }
  for (const auto & joint_name : params_.steer_names)
  {
    conf_names.push_back(joint_name + "/" + HW_IF_VELOCITY);
  }
  RCLCPP_WARN(get_node()->get_logger(), "Command interfaces set.");

  return {interface_configuration_type::INDIVIDUAL, conf_names};
  
}

InterfaceConfiguration FourSteeringController::state_interface_configuration() const
{
  std::vector<std::string> conf_joint_names;
  for (const auto & joint_name : params_.wheel_names)
  {
    conf_joint_names.push_back(joint_name + "/" + feedback_type());
  }
  for (const auto & joint_name : params_.steer_names)
  {
    conf_joint_names.push_back(joint_name + "/" + feedback_type());
  }
    RCLCPP_WARN(get_node()->get_logger(), "State interfaces set.");

  return {interface_configuration_type::INDIVIDUAL, conf_joint_names};
}


controller_interface::return_type FourSteeringController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  auto logger = get_node()->get_logger();
  if (get_state().id() == State::PRIMARY_STATE_INACTIVE)
  {
    if (!is_halted)
    {
      halt();
      is_halted = true;
    }
    return controller_interface::return_type::OK;
  }
  std::shared_ptr<Twist> last_command_msg;
  received_velocity_msg_ptr_.get(last_command_msg);

  if (last_command_msg == nullptr)
  {
    RCLCPP_WARN(logger, "Velocity message received was a nullptr.");
    return controller_interface::return_type::ERROR;
  }

  const auto age_of_last_command = time - last_command_msg->header.stamp;

  // Brake if cmd_vel has timeout, override the stored command
  if (age_of_last_command > cmd_vel_timeout_)
  {
    last_command_msg->twist.linear.x = 0.0;
    last_command_msg->twist.angular.z = 0.0;
  }
  Twist command = *last_command_msg;
  double & linear_cmd = command.twist.linear.x;
  double & angular_cmd = command.twist.angular.z;
  
  // check if OpenLoop condition is met.
  RCLCPP_WARN(get_node()->get_logger(), "Twist Correctly stored.");

  const double steering_track = params_.track_ - 2 * params_.wheel_steering_y_offset_;
  RCLCPP_WARN(get_node()->get_logger(), "Steering track value: %f", steering_track);


  const double joint_number = params_.wheel_names.size();
  double i = -1.0;
  // Compute wheels velocities:
  if(fabs(linear_cmd) > 0.001)
  {
    const double vel_steering_offset = (angular_cmd * params_.wheel_steering_y_offset_)/params_.wheel_radius_;
    const double sign = copysign(1.0, linear_cmd);
    RCLCPP_WARN(get_node()->get_logger(), "vel_steering_offset: %f", vel_steering_offset);

    for(size_t wheel=0; wheel < joint_number; wheel++)
    { //front_l - front_r - rear_l - rear_r
      wheel_vel_cmd[wheel] = sign * std::hypot((linear_cmd + (i) * angular_cmd*steering_track/2),
                                  (params_.wheel_base_*angular_cmd/2.0)) / params_.wheel_radius_ + (i) * vel_steering_offset;  
      i *= -1.0;
      RCLCPP_WARN(get_node()->get_logger(), "Wheel_cmd[%d]: %f", i,wheel_vel_cmd[wheel]);

    }
  }

  // Compute steering angles
  if(fabs(2.0*linear_cmd) > fabs(angular_cmd*steering_track))
  {
    for(size_t steer=0; steer < joint_number; steer++) {
      // front_l - rear_l - front_r - rear_r
      steer_vel_cmd[steer] = (-i) * atan(angular_cmd*params_.wheel_base_ /
                                      (2.0 * linear_cmd + (i) * angular_cmd * steering_track));
      i *= -1.0;
    }
  }

  // Set wheels velocities:
  for (size_t index=0; index < joint_number; index++)
  {
    registered_wheel_handles_[index].velocity.get().set_value(wheel_vel_cmd[index]);
  }
  // Set steers velocities:
  for (size_t index=0; index < joint_number; index++)
  {
    registered_steer_handles_[index].velocity.get().set_value(steer_vel_cmd[index]);
  }
  return controller_interface::return_type::OK;

}

controller_interface::CallbackReturn FourSteeringController::on_configure(
  const rclcpp_lifecycle::State &)
{
  auto logger = get_node()->get_logger();

  // update parameters if they have changed
  if (param_listener_->is_old(params_))
  {
    params_ = param_listener_->get_params();
    RCLCPP_INFO(logger, "Parameters were updated");
  }

  if (params_.wheel_names.empty())
  {
    RCLCPP_ERROR(
      logger, "The wheels parameter can't be empty. Wheels_names size: [%zu]", params_.wheel_names.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  const double wheel_separation = params_.track_;
  const double wheel_radius = params_.wheel_radius_;
  const double wheel_steering_y_offset_ = params_.wheel_steering_y_offset_;
  
  if (!reset())
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  const Twist empty_twist;
  received_velocity_msg_ptr_.set(std::make_shared<Twist>(empty_twist));
  RCLCPP_WARN(get_node()->get_logger(), "Empty Twist Command created.");
  // Fill last two commands with default constructed commands
  previous_commands_.emplace(empty_twist);
  previous_commands_.emplace(empty_twist);

  // initialize command subscriber 
  velocity_command_unstamped_subscriber_ =
    get_node()->create_subscription<geometry_msgs::msg::Twist>(
      DEFAULT_COMMAND_UNSTAMPED_TOPIC, rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<geometry_msgs::msg::Twist> msg) -> void
      {
        RCLCPP_WARN(get_node()->get_logger(), "Entered the lambda function.");

        if (!subscriber_is_active_)
        {
          RCLCPP_WARN(
            get_node()->get_logger(), "Can't accept new commands. subscriber is inactive");
          return;
        }
        // Write fake header in the stored stamped command
        std::shared_ptr<Twist> twist_stamped;
        received_velocity_msg_ptr_.get(twist_stamped);
        twist_stamped->twist = *msg;
        twist_stamped->header.stamp = get_node()->get_clock()->now();
        std::cout << "Msg created all good." << std::endl;

      }
    );

  previous_update_timestamp_ = get_node()->get_clock()->now();

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FourSteeringController::on_activate(const rclcpp_lifecycle::State &)
{
  std::cout << "On_activate: configuring wheels and steers. " << std::endl;

  const auto wheels_result =
    configure_wheels(params_.wheel_names, registered_wheel_handles_);
  const auto steers_result =
    configure_steers(params_.steer_names, registered_steer_handles_);

  if (
    wheels_result == controller_interface::CallbackReturn::ERROR ||
    steers_result == controller_interface::CallbackReturn::ERROR)
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  if (registered_wheel_handles_.empty() || registered_steer_handles_.empty())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Either wheel interfaces, steer interfaces are non existent");
    return controller_interface::CallbackReturn::ERROR;
  }

  is_halted = false;
  subscriber_is_active_ = true;

  RCLCPP_DEBUG(get_node()->get_logger(), "Subscriber and publisher are now active.");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FourSteeringController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  subscriber_is_active_ = false;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FourSteeringController::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  if (!reset())
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  received_velocity_msg_ptr_.set(std::make_shared<Twist>());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FourSteeringController::on_error(const rclcpp_lifecycle::State &)
{
  if (!reset())
  {
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

bool FourSteeringController::reset()
{

  // release the old queue
  std::queue<Twist> empty;
  std::swap(previous_commands_, empty);
  registered_wheel_handles_.clear();
  registered_steer_handles_.clear();

  subscriber_is_active_ = false;
  velocity_command_subscriber_.reset();
  velocity_command_unstamped_subscriber_.reset();

  received_velocity_msg_ptr_.set(nullptr);
  is_halted = false;

  return true;
}

controller_interface::CallbackReturn FourSteeringController::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

void FourSteeringController::halt()
{
  const auto halt_wheels = [](auto & wheel_handles)
  {
    for (const auto & wheel_handle : wheel_handles)
    {
      wheel_handle.velocity.get().set_value(0.0);
    }
  };

  halt_wheels(registered_wheel_handles_);
  
}

controller_interface::CallbackReturn FourSteeringController::configure_wheels(
  const std::vector<std::string> & wheel_names,
  std::vector<WheelHandle> & registered_handles)
{
  auto logger = get_node()->get_logger();

  if (wheel_names.empty())
  {
    RCLCPP_ERROR(logger, "No  wheel names specified");
    return controller_interface::CallbackReturn::ERROR;
  }

  // register handles
  registered_handles.reserve(wheel_names.size());
  for (const auto & wheel_name : wheel_names)
  {
    const auto interface_name = feedback_type();
    const auto state_handle = std::find_if(
      state_interfaces_.cbegin(), state_interfaces_.cend(),
      [&wheel_name, &interface_name](const auto & interface)
      {
        return interface.get_prefix_name() == wheel_name &&
               interface.get_interface_name() == interface_name;
      });

    if (state_handle == state_interfaces_.cend())
    {
      RCLCPP_ERROR(logger, "Unable to obtain joint state handle for %s", wheel_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }

    const auto command_handle = std::find_if(
      command_interfaces_.begin(), command_interfaces_.end(),
      [&wheel_name](const auto & interface)
      {
        return interface.get_prefix_name() == wheel_name &&
               interface.get_interface_name() == HW_IF_VELOCITY;
      });

    if (command_handle == command_interfaces_.end())
    {
      RCLCPP_ERROR(logger, "Unable to obtain joint command handle for %s", wheel_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }

    registered_handles.emplace_back(
      WheelHandle{std::ref(*state_handle), std::ref(*command_handle)});
  }

  return controller_interface::CallbackReturn::SUCCESS;
}


controller_interface::CallbackReturn FourSteeringController::configure_steers(
  const std::vector<std::string> & steer_names,
  std::vector<SteerHandle> & registered_handles)
{
  auto logger = get_node()->get_logger();

  if (steer_names.empty())
  {
    RCLCPP_ERROR(logger, "No steer names specified");
    return controller_interface::CallbackReturn::ERROR;
  }

  // register handles
  registered_handles.reserve(steer_names.size());
  for (const auto & steer_name : steer_names)
  {
    const auto interface_name = feedback_type();
    const auto state_handle = std::find_if(
      state_interfaces_.cbegin(), state_interfaces_.cend(),
      [&steer_name, &interface_name](const auto & interface)
      {
        return interface.get_prefix_name() == steer_name &&
               interface.get_interface_name() == interface_name;
      });

    if (state_handle == state_interfaces_.cend())
    {
      RCLCPP_ERROR(logger, "Unable to obtain joint state handle for %s", steer_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }

    const auto command_handle = std::find_if(
      command_interfaces_.begin(), command_interfaces_.end(),
      [&steer_name](const auto & interface)
      {
        return interface.get_prefix_name() == steer_name &&
               interface.get_interface_name() == HW_IF_VELOCITY;
      });

    if (command_handle == command_interfaces_.end())
    {
      RCLCPP_ERROR(logger, "Unable to obtain joint command handle for %s", steer_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }

    registered_handles.emplace_back(
      SteerHandle{std::ref(*state_handle), std::ref(*command_handle)});
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

} // namespace four_steering_controller
#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  four_steering_controller::FourSteeringController, controller_interface::ControllerInterface)
