#ifndef INVERSE_KINEMATICS_HPP_
#define INVERSE_KINEMATICS_HPP_

#include "steering_controllers_library_parameters.hpp"
#include <vector>

class InverseKinematics {
public:
    explicit InverseKinematics(const steering_controllers_library::Params& params) : params_(params) {}
    std::pair<std::vector<double>, std::vector<double>> calculateCommands(double linear_velocity, double angular_velocity);

private:
    steering_controllers_library::Params params_;
};

#endif  // INVERSE_KINEMATICS_HPP_
