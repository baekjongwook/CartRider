#pragma once

#include <algorithm>

// linear_vel  : m/s
// angular_vel : rad/s
// wheel_radius: m
// wheel_separation: m
// output wheel angular velocity: rad/s

namespace vehicle_kinematics
{

struct DifferentialOutput
{
    double left_w;  
    double right_w; 
};

struct ForwardOutput
{
    double linear_vel; 
    double angular_vel; 
};

class DifferentialDrive
{
public:
    DifferentialDrive(double wheel_radius, double wheel_separation, double max_wheel_angular_vel)
    : wheel_radius_(wheel_radius), wheel_separation_(wheel_separation), max_wheel_w_(max_wheel_angular_vel)
    {}

    DifferentialOutput compute(double linear_vel, double angular_vel) const
    {
        double v_left  = linear_vel - (angular_vel * wheel_separation_ / 2.0);
        double v_right = linear_vel + (angular_vel * wheel_separation_ / 2.0);

        double w_left  = v_left  / wheel_radius_;
        double w_right = v_right / wheel_radius_;

        w_left  = std::clamp(w_left,  -max_wheel_w_, max_wheel_w_);
        w_right = std::clamp(w_right, -max_wheel_w_, max_wheel_w_);

        return {w_left, w_right};
    }

    ForwardOutput computeForward(double left_w, double right_w) const
    {
        double v_left  = left_w  * wheel_radius_;
        double v_right = right_w * wheel_radius_;

        double linear  = (v_left + v_right) / 2.0;
        double angular = (v_right - v_left) / wheel_separation_;

        return {linear, angular};
    }

private:
    double wheel_radius_;
    double wheel_separation_;
    double max_wheel_w_; 

};

}