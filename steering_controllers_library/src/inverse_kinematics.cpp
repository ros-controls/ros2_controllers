#include "inverse_kinematics.hpp"

std::pair<std::vector<double>, std::vector<double>> InverseKinematics::calculateCommands(double linear_velocity, double angular_velocity) {
    std::vector<double> traction_commands(params_.rear_wheels_names.size(), 0.0); 
    std::vector<double> steering_commands(params_.front_wheels_names.size(), 0.0);

    double wheel_base = params_.wheel_base;  
    double track_width = params_.track_width;  

    double radius = (angular_velocity != 0.0) ? (linear_velocity / angular_velocity) : std::numeric_limits<double>::infinity();

    for (size_t i = 0; i < traction_commands.size(); ++i) {
        traction_commands[i] = linear_velocity;  
    }

    for (size_t i = 0; i < steering_commands.size(); ++i) {
        steering_commands[i] = std::atan2(wheel_base, radius);  
    }

    return std::make_pair(traction_commands, steering_commands);
}
