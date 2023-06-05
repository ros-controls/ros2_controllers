#ifndef FOUR_STEERING_CONTROLLER__ODOMETRY_HPP_
#define FOUR_STEERING_CONTROLLER__ODOMETRY_HPP_

#include <cmath>

#include "rclcpp/time.hpp"
#include "rcppmath/rolling_mean_accumulator.hpp"

namespace four_steering_controller
{
class Odometry
{
public:
  explicit Odometry(size_t velocity_rolling_window_size = 10);

  void init(const rclcpp::Time & time);
  bool update(const double &fl_speed, const double &fr_speed,
              const double &rl_speed, const double &rr_speed,
              double front_steering, double rear_steering, const rclcpp::Time &time);
  bool updateFromVelocity(double left_vel, double right_vel, const rclcpp::Time & time);
  void updateOpenLoop(double linear, double angular, const rclcpp::Time & time);
  void resetOdometry();

  double getX() const { return x_; }
  double getY() const { return y_; }
  double getHeading() const { return heading_; }
  double getLinear() const { return linear_; }
  double getLinearX() const { return linear_x_; }
  double getLinearY() const { return linear_y_; }

  double getAngular() const { return angular_; }

  void setWheelParams( double steering_track, double wheel_radius,
                       double wheel_base, double wheel_steering_y_offset);
  void setVelocityRollingWindowSize(size_t velocity_rolling_window_size);

private:
  using RollingMeanAccumulator = rcppmath::RollingMeanAccumulator<double>;
  void integrateXY(double linear_x, double linear_y, double angular);
  void integrateRungeKutta2(double linear, double angular);
  void integrateExact(double linear, double angular);
  void resetAccumulators();

  // Current timestamp:
  rclcpp::Time timestamp_;

  // Current pose:
  double x_;        //   [m]
  double y_;        //   [m]
  double heading_;  // [rad]

  // Current velocity:
  double linear_;   //   [m/s]
  double angular_;  // [rad/s]
  double linear_x_;
  double linear_y_;
  // Wheel kinematic parameters [m]:
  double wheel_separation_;
  double wheel_radius_;
  double wheel_steering_y_offset_;
  double wheel_base_;
  double steering_track_;
  // Previous wheel position/state [rad]:
  double wheel_old_pos_;

  // Rolling mean accumulators for the linear and angular velocities:
  size_t velocity_rolling_window_size_;
  RollingMeanAccumulator linear_accumulator_;
  RollingMeanAccumulator angular_accumulator_;
};

}  // namespace four_steering_controller

#endif  // FOUR_STEERING_CONTROLLER__ODOMETRY_HPP_
