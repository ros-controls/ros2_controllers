#include "forward_kinematics.hpp"

struct Odometry {
    double x;
    double y;
    double theta;
};

Odometry ForwardKinematics::calculate(double linear_velocity, double angular_velocity) {
    // Implement your forward kinematics calculation logic here
    // Example placeholder calculation
    Odometry odom_result;
    odom_result.x = linear_velocity * params_.wheel_base;  // Replace with correct formula
    odom_result.y = 0.0;  // Adjust if side motion is modeled
    odom_result.theta = angular_velocity;  // Adjust if needed

    return odom_result;
}