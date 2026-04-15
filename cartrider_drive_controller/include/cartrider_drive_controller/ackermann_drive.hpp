#pragma once

#include <cmath>

// linear_vel   : m/s
// angular_vel  : rad/s
// wheel_base   : m   (front axle center <-> rear axle center distance)
// track_width  : m   (left wheel center <-> right wheel center distance)
// wheel_radius : m
//
// output:
// - left_steer, right_steer : rad
// - left_w, right_w         : rad/s
//
// notes:
// - This model assumes front-wheel steering Ackermann geometry.
// - Wheel speed sign follows linear_vel sign.
// - Pure spin-in-place is not supported in Ackermann mode.

namespace vehicle_kinematics
{

    struct AckermannOutput
    {
        double left_steer;
        double right_steer;
        double left_w;
        double right_w;
    };

    class AckermannDrive
    {
    public:
        AckermannDrive(double wheel_base, double track_width, double wheel_radius)
            : wheel_base_(wheel_base), track_width_(track_width), wheel_radius_(wheel_radius)
        {
        }

        AckermannOutput compute(double linear_vel, double angular_vel) const
        {
            constexpr double kLinearEps = 1e-6;
            constexpr double kAngularEps = 1e-6;

            AckermannOutput out{};

            if (std::abs(linear_vel) < kLinearEps)
            {
                out.left_steer = 0.0;
                out.right_steer = 0.0;
                out.left_w = 0.0;
                out.right_w = 0.0;
                return out;
            }

            if (std::abs(angular_vel) < kAngularEps)
            {
                const double wheel_radps = linear_vel / wheel_radius_;

                out.left_steer = 0.0;
                out.right_steer = 0.0;
                out.left_w = wheel_radps;
                out.right_w = wheel_radps;
                return out;
            }

            const double turn_radius = linear_vel / angular_vel;

            const double left_turn_radius = turn_radius - (track_width_ / 2.0);
            const double right_turn_radius = turn_radius + (track_width_ / 2.0);

            out.left_steer = std::atan(wheel_base_ / left_turn_radius);
            out.right_steer = std::atan(wheel_base_ / right_turn_radius);

            const double left_path_radius =
                std::sqrt((left_turn_radius * left_turn_radius) + (wheel_base_ * wheel_base_));

            const double right_path_radius =
                std::sqrt((right_turn_radius * right_turn_radius) + (wheel_base_ * wheel_base_));

            const double turn_radius_abs = std::abs(turn_radius);

            const double left_linear_vel =
                std::copysign(std::abs(linear_vel) * (left_path_radius / turn_radius_abs), linear_vel);

            const double right_linear_vel =
                std::copysign(std::abs(linear_vel) * (right_path_radius / turn_radius_abs), linear_vel);

            out.left_w = left_linear_vel / wheel_radius_;
            out.right_w = right_linear_vel / wheel_radius_;

            return out;
        }

    private:
        double wheel_base_;
        double track_width_;
        double wheel_radius_;
    };

}