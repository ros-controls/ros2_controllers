#ifndef FOUR_WHEEL_STEERING_CONTROLLER__FOUR_WHEEL_STEERING_CONTROLLER_HPP_
#define FOUR_WHEEL_STEERING_CONTROLLER__FOUR_WHEEL_STEERING_CONTROLLER_HPP_

#include <memory>

#include "four_wheel_steering_controller/visibility_control.h"
#include "four_wheel_steering_controller_parameters.hpp"
#include "steering_controllers_library/steering_controllers_library.hpp"

namespace four_wheel_steering_controller
{
// name constants for state interfaces
static constexpr size_t STATE_TRACTION_FRONT_RIGHT_WHEEL = 0;
static constexpr size_t STATE_TRACTION_FRONT_LEFT_WHEEL = 1;
static constexpr size_t STATE_TRACTION_REAR_RIGHT_WHEEL = 2;
static constexpr size_t STATE_TRACTION_REAR_LEFT_WHEEL = 3;

static constexpr size_t STATE_STEER_FRONT_RIGHT_WHEEL = 4;
static constexpr size_t STATE_STEER_FRONT_LEFT_WHEEL = 5;
static constexpr size_t STATE_STEER_REAR_RIGHT_WHEEL = 6;
static constexpr size_t STATE_STEER_REAR_LEFT_WHEEL = 7;

// name constants for command interfaces
static constexpr size_t CMD_TRACTION_FRONT_RIGHT_WHEEL = 0;
static constexpr size_t CMD_TRACTION_FRONT_LEFT_WHEEL = 1;
static constexpr size_t CMD_TRACTION_REAR_RIGHT_WHEEL = 2;
static constexpr size_t CMD_TRACTION_REAR_LEFT_WHEEL = 3;

static constexpr size_t CMD_STEER_FRONT_RIGHT_WHEEL = 4;
static constexpr size_t CMD_STEER_FRONT_LEFT_WHEEL = 5;
static constexpr size_t CMD_STEER_REAR_RIGHT_WHEEL = 6;
static constexpr size_t CMD_STEER_REAR_LEFT_WHEEL = 7;

static constexpr size_t NR_STATE_ITFS = 8;
static constexpr size_t NR_CMD_ITFS = 8;
static constexpr size_t NR_REF_ITFS = 2;

class FourWheelSteeringController : public steering_controllers_library::SteeringControllersLibrary
{
public:
  FourWheelSteeringController();

  FOUR_WHEEL_STEERING_CONTROLLER__VISIBILITY_PUBLIC controller_interface::CallbackReturn
  configure_odometry() override;

  FOUR_WHEEL_STEERING_CONTROLLER__VISIBILITY_PUBLIC bool update_odometry(
    const rclcpp::Duration & period) override;

  FOUR_WHEEL_STEERING_CONTROLLER__VISIBILITY_PUBLIC void
  initialize_implementation_parameter_listener() override;

protected:
  std::shared_ptr<four_wheel_steering_controller::ParamListener> four_wheel_steering_param_listener_;
  four_wheel_steering_controller::Params four_wheel_steering_param_;
};
}  // namespace four_wheel_steering_controller

#endif  // FOUR_WHEEL_STEERING_CONTROLLER__FOUR_WHEEL_STEERING_CONTROLLER_HPP_
