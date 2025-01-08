#ifndef FORWARD_KINEMATICS_HPP_
#define FORWARD_KINEMATICS_HPP_

#include "steering_controllers_library_parameters.hpp"

class ForwardKinematics {
public:
    explicit ForwardKinematics(const steering_controllers_library::Params& params) : params_(params) {}
    Odometry calculate(double linear_velocity, double angular_velocity);  // Implement forward calculation

private:
    steering_controllers_library::Params params_;
};

#endif  // FORWARD_KINEMATICS_HPP_
