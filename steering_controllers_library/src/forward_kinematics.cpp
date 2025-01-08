#include "forward_kinematics.hpp"

struct Odometry {
    double x;
    double y;
    double theta;
};

Odometry ForwardKinematics::calculate(double linear_velocity, double angular_velocity) {
    
    Odometry odom_result;
    odom_result.x = linear_velocity * params_.wheel_base;  
    odom_result.y = 0.0;  
    odom_result.theta = angular_velocity;  

    return odom_result;
}