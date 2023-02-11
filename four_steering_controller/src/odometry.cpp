#include "four_steering_controller/odometry.hpp"

namespace four_steering_controller
{
Odometry::Odometry(size_t velocity_rolling_window_size)
: timestamp_(0.0),
  x_(0.0),
  y_(0.0),
  heading_(0.0),
  linear_(0.0),
  linear_x_(0.0),
  linear_y_(0.0),
  angular_(0.0),
  wheel_separation_(0.0),
  wheel_radius_(0.0),
  wheel_base_(0.0),
  wheel_steering_y_offset_(0.0),
  steering_track_(0.0),
  wheel_old_pos_(0.0),
  velocity_rolling_window_size_(velocity_rolling_window_size),
  linear_accumulator_(velocity_rolling_window_size),
  angular_accumulator_(velocity_rolling_window_size)
{
}

void Odometry::init(const rclcpp::Time & time)
{
  // Reset accumulators and timestamp:
  resetAccumulators();
  timestamp_ = time;
}

bool Odometry::update(const double &fl_speed, const double &fr_speed,
                      const double &rl_speed, const double &rr_speed,
                      double front_steering, double rear_steering, const rclcpp::Time &time)
{
  // We cannot estimate the speed with very small time intervals:
  const double dt = time.seconds() - timestamp_.seconds();
  if (dt < 0.0001)
  {
    return false;  // Interval too small to integrate with
  }

  const double front_tmp = cos(front_steering)*(tan(front_steering)-tan(rear_steering))/wheel_base_;
  const double front_left_tmp = front_tmp/sqrt(1-steering_track_*front_tmp*cos(front_steering) + pow(steering_track_*front_tmp/2,2));
  const double front_right_tmp = front_tmp/sqrt(1+steering_track_*front_tmp*cos(front_steering) + pow(steering_track_*front_tmp/2,2));
  const double fl_speed_tmp = fl_speed * (1/(1-wheel_steering_y_offset_*front_left_tmp));
  const double fr_speed_tmp = fr_speed * (1/(1-wheel_steering_y_offset_*front_right_tmp));
  const double front_linear_speed = wheel_radius_ * copysign(1.0, fl_speed_tmp+fr_speed_tmp) * sqrt((pow(fl_speed,2)+pow(fr_speed,2))/(2+pow(steering_track_*front_tmp,2)/2.0));

  const double rear_tmp = cos(rear_steering)*(tan(front_steering)-tan(rear_steering))/wheel_base_;
  const double rear_left_tmp = rear_tmp/sqrt(1-steering_track_*rear_tmp*cos(rear_steering) + pow(steering_track_*rear_tmp/2,2));
  const double rear_right_tmp = rear_tmp/sqrt(1+steering_track_*rear_tmp*cos(rear_steering) + pow(steering_track_*rear_tmp/2,2));
  const double rl_speed_tmp = rl_speed * (1/(1-wheel_steering_y_offset_*rear_left_tmp));
  const double rr_speed_tmp = rr_speed * (1/(1-wheel_steering_y_offset_*rear_right_tmp));
  const double rear_linear_speed = wheel_radius_ * copysign(1.0, rl_speed_tmp+rr_speed_tmp) * sqrt((pow(rl_speed_tmp,2)+pow(rr_speed_tmp,2)) / (2+pow(steering_track_*rear_tmp,2)/2.0));

  angular_ = (front_linear_speed*front_tmp + rear_linear_speed*rear_tmp)/2.0;

  linear_x_ = (front_linear_speed*cos(front_steering) + rear_linear_speed*cos(rear_steering))/2.0;
  linear_y_ = (front_linear_speed*sin(front_steering) - wheel_base_*angular_/2.0 + rear_linear_speed*sin(rear_steering) + wheel_base_*angular_/2.0)/2.0;
  linear_ =  copysign(1.0, rear_linear_speed)*sqrt(pow(linear_x_,2)+pow(linear_y_,2));

  timestamp_ = time;
  /// Integrate odometry:
  integrateXY(linear_x_*dt, linear_y_*dt, angular_*dt);

  return true;
}

bool Odometry::updateFromVelocity(double left_vel, double right_vel, const rclcpp::Time & time)
{
  const double dt = time.seconds() - timestamp_.seconds();

  // Compute linear and angular diff:
  const double linear = (left_vel + right_vel) * 0.5;
  // Now there is a bug about scout angular velocity
  const double angular = (right_vel - left_vel) / wheel_separation_;

  // Integrate odometry:
  integrateExact(linear, angular);

  timestamp_ = time;

  // Estimate speeds using a rolling mean to filter them out:
  linear_accumulator_.accumulate(linear / dt);
  angular_accumulator_.accumulate(angular / dt);

  linear_ = linear_accumulator_.getRollingMean();
  angular_ = angular_accumulator_.getRollingMean();

  return true;
}

void Odometry::updateOpenLoop(double linear, double angular, const rclcpp::Time & time)
{
  /// Save last linear and angular velocity:
  linear_ = linear;
  angular_ = angular;

  /// Integrate odometry:
  const double dt = time.seconds() - timestamp_.seconds();
  timestamp_ = time;
  integrateExact(linear * dt, angular * dt);
}

void Odometry::resetOdometry()
{
  x_ = 0.0;
  y_ = 0.0;
  heading_ = 0.0;
}

void Odometry::integrateXY(double linear_x, double linear_y, double angular)
{
  const double delta_x = linear_x*cos(heading_) - linear_y*sin(heading_);
  const double delta_y = linear_x*sin(heading_) + linear_y*cos(heading_);

  x_ += delta_x;
  y_ += delta_y;
  heading_ += angular;
}

void Odometry::setWheelParams(
  double steering_track, double wheel_radius, double wheel_base, double wheel_steering_y_offset)
{
  steering_track_   = steering_track;
  wheel_steering_y_offset_ = wheel_steering_y_offset;
  wheel_radius_     = wheel_radius;
  wheel_base_       = wheel_base;
}

void Odometry::setVelocityRollingWindowSize(size_t velocity_rolling_window_size)
{
  velocity_rolling_window_size_ = velocity_rolling_window_size;

  resetAccumulators();
}

void Odometry::integrateRungeKutta2(double linear, double angular)
{
  const double direction = heading_ + angular * 0.5;

  /// Runge-Kutta 2nd order integration:
  x_ += linear * cos(direction);
  y_ += linear * sin(direction);
  heading_ += angular;
}

void Odometry::integrateExact(double linear, double angular)
{
  if (fabs(angular) < 1e-6)
  {
    integrateRungeKutta2(linear, angular);
  }
  else
  {
    /// Exact integration (should solve problems when angular is zero):
    const double heading_old = heading_;
    const double r = linear / angular;
    heading_ += angular;
    x_ += r * (sin(heading_) - sin(heading_old));
    y_ += -r * (cos(heading_) - cos(heading_old));
  }
}

void Odometry::resetAccumulators()
{
  linear_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
  angular_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
}

}  // namespace four_steering_controller
