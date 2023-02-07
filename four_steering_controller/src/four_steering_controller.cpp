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

controller_interface::CallbackReturn FourSteeringController::on_init()
{
  try
  {
    // Create the parameter listener and get the parameters
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
    RCLCPP_WARN(get_node()->get_logger(), "Parameters read correctly.");

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
    conf_names.push_back(joint_name + "/" + HW_IF_POSITION);
  }
  RCLCPP_WARN(get_node()->get_logger(), "Command interfaces set.");

  return {interface_configuration_type::INDIVIDUAL, conf_names};
  
}

InterfaceConfiguration FourSteeringController::state_interface_configuration() const
{
  std::vector<std::string> conf_joint_names;
  for (const auto & joint_name : params_.wheel_names)
  {
    conf_joint_names.push_back(joint_name + "/" + HW_IF_VELOCITY);
  }
  for (const auto & joint_name : params_.steer_names)
  {
    conf_joint_names.push_back(joint_name + "/" + HW_IF_POSITION);
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
  //std::shared_ptr<Twist> last_command_msg;

  std::shared_ptr<four_wheel_steering_msgs::msg::FourWheelSteeringStamped> last_command_msg;
  received_velocity_msg_ptr_.get(last_command_msg);

  if (last_command_msg == nullptr)
  {
    RCLCPP_WARN(logger, "Velocity message received was a nullptr.");
    return controller_interface::return_type::ERROR;
  }

  // Brake if cmd_vel has timeout, override the stored command
  const auto age_of_last_command = time - last_command_msg->header.stamp;
  if (age_of_last_command > cmd_vel_timeout_)
  {
    last_command_msg->data.speed = 0.0;
    last_command_msg->data.front_steering_angle = 0.0;
    last_command_msg->data.rear_steering_angle = 0.0;
    last_command_msg->data.acceleration = 0.0;
  }
  // Save twist command:
/*   Twist command = *last_command_msg;
  double & linear_cmd_x = command.twist.linear.x;
  double & linear_cmd_y = command.twist.linear.y;
  double & angular_cmd = command.twist.angular.z;
 */

  four_wheel_steering_msgs::msg::FourWheelSteeringStamped  curr_cmd_4ws = *last_command_msg;
  const double steering_track = params_.track_ - 2 * params_.wheel_steering_y_offset_;
  const double joint_number = params_.wheel_names.size();
  //const double sign = copysign(1.0, linear_cmd_x);
/* 
  if(fabs(linear_cmd_x) > 0.001)
  {
    const double vel_steering_offset = (angular_cmd * params_.wheel_steering_y_offset_)/params_.wheel_radius_;

    vel_left_front = sign * std::hypot((linear_cmd_x - angular_cmd * steering_track/2),
                                        (params_.wheel_base_*angular_cmd/2.0)) / params_.wheel_radius_
                      - vel_steering_offset;
    wheel_vel_cmd[0] = vel_left_front;

    vel_right_front = sign * std::hypot((linear_cmd_x + angular_cmd * steering_track/2),
                                        (params_.wheel_base_*angular_cmd/2.0)) / params_.wheel_radius_
                      + vel_steering_offset;
    wheel_vel_cmd[1] = vel_right_front;

    vel_left_rear = sign * std::hypot((linear_cmd_x - angular_cmd * steering_track/2),
                                        (params_.wheel_base_*angular_cmd/2.0)) / params_.wheel_radius_
                      - vel_steering_offset;
    wheel_vel_cmd[2] = vel_left_rear;

    vel_right_rear = sign * std::hypot((linear_cmd_x + angular_cmd * steering_track/2),
                                      (params_.wheel_base_*angular_cmd/2.0)) / params_.wheel_radius_
                      + vel_steering_offset;
    wheel_vel_cmd[3] = vel_right_rear;
    
  }
  // Compute steering angles
  //if(fabs(linear_cmd_x) > 0.001 && fabs(angular_cmd) > 0.001 )
  //if(fabs(2.0*linear_cmd_x) > fabs(angular_cmd*steering_track) && fabs(linear_cmd_y) < 0.001)
  //if(fabs(linear_cmd_x) > 0.001 && fabs(angular_cmd) > 0.001)
  if(fabs(linear_cmd_x) > 0.001)
  {
    if(fabs(angular_cmd) > 2*fabs(linear_cmd_x)*steering_track)
    {
      float sign = copysign(1, angular_cmd);
      angular_cmd = sign * 2*fabs(linear_cmd_x)*steering_track;
    }

    front_left_steering = atan(angular_cmd * params_.wheel_base_ /
                                (2.0 * linear_cmd_x - angular_cmd * steering_track));
    steer_cmd[0] = front_left_steering;

    front_right_steering = atan(angular_cmd * params_.wheel_base_ /
                                  (2.0 * linear_cmd_x + angular_cmd * steering_track));
    steer_cmd[1] = front_right_steering;
  
    rear_left_steering = -front_left_steering;
    rear_right_steering = -front_right_steering;
    steer_cmd[2] = rear_left_steering;
    steer_cmd[3] = rear_right_steering;
  }
 else if(fabs(linear_cmd_x) > 0.001) //Straight motion
  {
    front_left_steering = copysign(M_PI_2, angular_cmd);
    steer_cmd[0] = front_left_steering;

    front_right_steering = copysign(M_PI_2, angular_cmd);
    steer_cmd[1] = front_right_steering;

    rear_left_steering = -front_left_steering;
    rear_right_steering = -front_right_steering;
    steer_cmd[2] = rear_left_steering;
    steer_cmd[3] = rear_right_steering;
  }

  
  // In-Phase steering:
  if(fabs(linear_cmd_x) > 0.001 && fabs(linear_cmd_y) >= 0.001 && fabs(angular_cmd) < 0.001)
  {
    wheel_vel = sign * std::hypot(linear_cmd_x, linear_cmd_y);
    wheel_vel_cmd[0] = wheel_vel;
    wheel_vel_cmd[1] = wheel_vel;
    wheel_vel_cmd[2] = wheel_vel;
    wheel_vel_cmd[3] = wheel_vel;

    front_left_steering = atan(linear_cmd_y / linear_cmd_x);
    steer_cmd[0] = front_left_steering;
    front_right_steering = atan(linear_cmd_y / linear_cmd_x);
    steer_cmd[1] = front_right_steering;
    rear_left_steering = front_left_steering;
    steer_cmd[2] = rear_left_steering;
    rear_right_steering = front_right_steering;        
    steer_cmd[3] = rear_right_steering;
    // FIXME missing wheel speed
  }
   else if(fabs(linear_cmd_x) < 0.001 && fabs(linear_cmd_y) < 0.001 && fabs(angular_cmd) > 0.001)   
  {
    // Pivot Steering:
    front_left_steering = -atan(params_.wheel_base_ / steering_track);
    steer_cmd[0] = front_left_steering;
    front_right_steering = atan(params_.wheel_base_ / steering_track);
    steer_cmd[1] = front_right_steering;
    rear_left_steering = atan(params_.wheel_base_ / steering_track);
    steer_cmd[2] = rear_left_steering;
    rear_right_steering = -atan(params_.wheel_base_ / steering_track);
    steer_cmd[3] = rear_right_steering;
    vel_left_front = -angular_cmd;
    vel_right_front = angular_cmd;
    wheel_vel_cmd[0] = vel_left_front;
    wheel_vel_cmd[1] = vel_right_front;
    wheel_vel_cmd[2] = -vel_left_front;
    wheel_vel_cmd[3] = -vel_right_front;
  }

  for(int index=0; index < joint_number; index++){
    alpha_read[index] = registered_steer_handles_[index].feedback_pos.get().get_value()*180.0/M_PI;
    std::cout << "Joint: " << index << "  " << "Angle: " << alpha_read[index] << std::endl;
  }
  std::cout << "===============" << std::endl;

  // Set wheels velocities:
  for (size_t index=0; index < joint_number; index++)
  {
    registered_wheel_handles_[index].velocity.get().set_value(wheel_vel_cmd[index]);
  }
 
  for(int index=0; index < joint_number; index++){
    w_speed[index] = registered_wheel_handles_[index].feedback.get().get_value();
    std::cout << "Wheel: " << index << "  " << "Speed: " << w_speed[index] << std::endl;
  }
  std::cout << "===============" << std::endl;
 */

  curr_cmd_4ws.data.front_steering_angle = std::clamp(curr_cmd_4ws.data.front_steering_angle, static_cast<float>(-M_PI_2/2), static_cast<float>(M_PI_2/2));
  curr_cmd_4ws.data.rear_steering_angle = std::clamp(curr_cmd_4ws.data.rear_steering_angle,  static_cast<float>(-M_PI_2/2), static_cast<float>(M_PI_2/2));

  // Compute steering angles
  const double tan_front_steering = tan(curr_cmd_4ws.data.front_steering_angle);
  const double tan_rear_steering  = tan(curr_cmd_4ws.data.rear_steering_angle);
  const double steering_diff =  steering_track*(tan_front_steering - tan_rear_steering)/2.0;
  if(fabs(params_.wheel_base_ - fabs(steering_diff)) > 0.001)
  {
    front_left_steering = atan(params_.wheel_base_*tan_front_steering/(params_.wheel_base_-steering_diff));
    steer_cmd[0] = front_left_steering;
    front_right_steering = atan(params_.wheel_base_*tan_front_steering/(params_.wheel_base_+steering_diff));
    steer_cmd[1] = front_right_steering; 
    rear_left_steering = atan(params_.wheel_base_*tan_rear_steering/(params_.wheel_base_-steering_diff));
    steer_cmd[2] = rear_left_steering;
    rear_right_steering = atan(params_.wheel_base_*tan_rear_steering/(params_.wheel_base_+steering_diff));
    steer_cmd[3] = rear_right_steering;
  }
  if(tan_front_steering == 1.0)
  {
    pivot = true;
    front_left_steering = -atan(params_.wheel_base_ / steering_track);
    steer_cmd[0] = front_left_steering;
    front_right_steering = atan(params_.wheel_base_ / steering_track);
    steer_cmd[1] = front_right_steering;
    rear_left_steering = atan(params_.wheel_base_ / steering_track);
    steer_cmd[2] = rear_left_steering;
    rear_right_steering = -atan(params_.wheel_base_ / steering_track);
    steer_cmd[3] = rear_right_steering;
  }
  // Compute wheels velocities:
  if(fabs(curr_cmd_4ws.data.speed) > 0.001 && !pivot)
  {
    //Virutal front and rear wheelbase
    // distance between the projection of the CIR on the wheelbase and the front axle
    double l_front = 0;
    if(fabs(tan(front_left_steering) - tan(front_right_steering)) > 0.01)
    {
      l_front = tan(front_right_steering) * tan(front_left_steering) * steering_track
          / (tan(front_left_steering) - tan(front_right_steering));
    }
    // distance between the projection of the CIR on the wheelbase and the rear axle
    double l_rear = 0;
    if(fabs(tan(rear_left_steering) - tan(rear_right_steering)) > 0.01)
    {
      l_rear = tan(rear_right_steering) * tan(rear_left_steering) * steering_track
          / (tan(rear_left_steering) - tan(rear_right_steering));
    }

    const double angular_speed_cmd = curr_cmd_4ws.data.speed * (tan_front_steering-tan_rear_steering)/params_.wheel_base_;
    const double vel_steering_offset = (angular_speed_cmd*params_.wheel_steering_y_offset_)/params_.wheel_radius_;
    const double sign = copysign(1.0, curr_cmd_4ws.data.speed);

    vel_left_front  = sign * std::hypot((curr_cmd_4ws.data.speed - angular_speed_cmd*steering_track/2),
                                        (l_front*angular_speed_cmd))/params_.wheel_radius_
                      - vel_steering_offset;
    wheel_vel_cmd[0] = vel_left_front;
    vel_right_front = sign * std::hypot((curr_cmd_4ws.data.speed + angular_speed_cmd*steering_track/2),
                                        (l_front*angular_speed_cmd))/params_.wheel_radius_
                      + vel_steering_offset;
    wheel_vel_cmd[1] = vel_right_front;
    vel_left_rear = sign * std::hypot((curr_cmd_4ws.data.speed - angular_speed_cmd*steering_track/2),
                                      (l_rear*angular_speed_cmd))/params_.wheel_radius_
                    - vel_steering_offset;
    wheel_vel_cmd[2] = vel_left_rear;
    vel_right_rear = sign * std::hypot((curr_cmd_4ws.data.speed + angular_speed_cmd*steering_track/2),
                                        (l_rear*angular_speed_cmd))/params_.wheel_radius_
                      + vel_steering_offset;
    wheel_vel_cmd[3] = vel_right_rear;
  }
  if(pivot)
  {
    vel_left_front = -curr_cmd_4ws.data.speed;
    vel_right_front = curr_cmd_4ws.data.speed;
    wheel_vel_cmd[0] = vel_left_front;
    wheel_vel_cmd[1] = vel_right_front;
    wheel_vel_cmd[2] = vel_left_front;
    wheel_vel_cmd[3] = vel_right_front;
    pivot = false;
  }
  //Send steering cmds:
  for (size_t index=0; index < joint_number; index++)
  {
    registered_steer_handles_[index].position.get().set_value(steer_cmd[index]);
  }
  //Send speed cmds:
  for (size_t index=0; index < joint_number; index++)
  {
    registered_wheel_handles_[index].velocity.get().set_value(wheel_vel_cmd[index]);
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

  if (!reset())
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  const four_wheel_steering_msgs::msg::FourWheelSteeringStamped empty_cmd;
  received_velocity_msg_ptr_.set(std::make_shared<four_wheel_steering_msgs::msg::FourWheelSteeringStamped>(empty_cmd));
  // Fill last two commands with default constructed commands
  previous_commands_.emplace(empty_cmd);
  previous_commands_.emplace(empty_cmd);

  // initialize command subscriber 
  velocity_command_unstamped_subscriber_ =
    get_node()->create_subscription<four_wheel_steering_msgs::msg::FourWheelSteeringStamped>(
      DEFAULT_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<four_wheel_steering_msgs::msg::FourWheelSteeringStamped> msg) -> void
      {
        if (!subscriber_is_active_)
        {
          RCLCPP_WARN(
            get_node()->get_logger(), "Can't accept new commands. subscriber is inactive");
          return;
        }
        if ((msg->header.stamp.sec == 0) && (msg->header.stamp.nanosec == 0))
        {
          RCLCPP_WARN_ONCE(
            get_node()->get_logger(),
            "Received TwistStamped with zero timestamp, setting it to current "
            "time, this message will only be shown once");
          msg->header.stamp = get_node()->get_clock()->now();
        }
        received_velocity_msg_ptr_.set(std::move(msg));
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
  RCLCPP_WARN(get_node()->get_logger(),"Configuration completed.");
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

  received_velocity_msg_ptr_.set(std::make_shared<four_wheel_steering_msgs::msg::FourWheelSteeringStamped>());
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
  std::queue<four_wheel_steering_msgs::msg::FourWheelSteeringStamped> empty;
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

  const auto halt_steers = [](auto & steer_handles)
  {
    for (const auto & steer_handle : steer_handles)
    {
      steer_handle.position.get().set_value(0.0);
    }
  };

  halt_wheels(registered_wheel_handles_);
  halt_steers(registered_steer_handles_);

}

controller_interface::CallbackReturn FourSteeringController::configure_wheels(
  const std::vector<std::string> & wheel_names,
  std::vector<WheelHandle> & registered_handles)
{
  RCLCPP_INFO(get_node()->get_logger(), "Get Wheel Joint Instance");

  if (wheel_names.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "No  wheel names specified");
    return controller_interface::CallbackReturn::ERROR;
  }

  // register handles
  registered_handles.reserve(wheel_names.size());
  for (const auto & wheel_name : wheel_names)
  {
    //const auto interface_name = feedback_type();
    const auto state_handle = std::find_if(
      state_interfaces_.cbegin(), state_interfaces_.cend(),
      [&wheel_name](const auto & interface)
      {
        return interface.get_prefix_name() == wheel_name &&
               interface.get_interface_name() == HW_IF_VELOCITY;
      });

    if (state_handle == state_interfaces_.cend())
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Unable to obtain joint state handle for %s", wheel_name.c_str());
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
      RCLCPP_ERROR(get_node()->get_logger(), "Unable to obtain joint command handle for %s", wheel_name.c_str());
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
  RCLCPP_INFO(get_node()->get_logger(), "Get Steering Joint Instance");

  if (steer_names.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "No steer names specified");
    return controller_interface::CallbackReturn::ERROR;
  }

  // register handles
  registered_handles.reserve(steer_names.size());
  for (const auto & steer_name : steer_names)
  {
    //const auto interface_name = feedback_type();
    const auto state_handle = std::find_if(
      state_interfaces_.cbegin(), state_interfaces_.cend(),
      [&steer_name](const auto & interface)
      {
        return interface.get_prefix_name() == steer_name &&
               interface.get_interface_name() == HW_IF_POSITION;
      });

    if (state_handle == state_interfaces_.cend())
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Unable to obtain joint state handle for %s", steer_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }

    const auto command_handle = std::find_if(
      command_interfaces_.begin(), command_interfaces_.end(),
      [&steer_name](const auto & interface)
      {
        return interface.get_prefix_name() == steer_name &&
               interface.get_interface_name() == HW_IF_POSITION;
      });

    if (command_handle == command_interfaces_.end())
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Unable to obtain joint command handle for %s", steer_name.c_str());
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
