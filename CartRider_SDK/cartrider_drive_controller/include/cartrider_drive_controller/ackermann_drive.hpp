#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace vehicle_kinematics
{

    struct TwoWSFourWDOutput
    {
        double front_left_steer;
        double front_right_steer;

        double front_left_w;
        double front_right_w;

        double rear_left_w;
        double rear_right_w;
    };

    class TwoWSFourWDDrive
    {
    public:
        TwoWSFourWDDrive(
            double wheel_base,
            double front_track_width,
            double rear_track_width,
            double front_wheel_radius,
            double rear_wheel_radius)
            : wheel_base_(wheel_base),
              front_track_width_(front_track_width),
              rear_track_width_(rear_track_width),
              front_wheel_radius_(front_wheel_radius),
              rear_wheel_radius_(rear_wheel_radius)
        {
            if (wheel_base_ <= 0.0 ||
                front_track_width_ <= 0.0 ||
                rear_track_width_ <= 0.0 ||
                front_wheel_radius_ <= 0.0 ||
                rear_wheel_radius_ <= 0.0)
            {
                throw std::invalid_argument("Invalid TwoWSFourWDDrive parameters");
            }
        }

        TwoWSFourWDOutput compute(double linear_vel, double angular_vel) const
        {
            constexpr double kMinLinearVel = 0.05;
            constexpr double kAngularEps = 1e-6;
            constexpr double kMaxCurvature = 1.0;

            TwoWSFourWDOutput out{};

            if (std::abs(linear_vel) < kMinLinearVel)
            {
                return out;
            }

            double effective_angular_vel = angular_vel;

            const double max_angular_vel = std::abs(linear_vel) * kMaxCurvature;

            effective_angular_vel = std::clamp(
                effective_angular_vel,
                -max_angular_vel,
                max_angular_vel);

            if (std::abs(effective_angular_vel) < kAngularEps)
            {
                out.front_left_steer = 0.0;
                out.front_right_steer = 0.0;

                out.front_left_w = linear_vel / front_wheel_radius_;
                out.front_right_w = linear_vel / front_wheel_radius_;

                out.rear_left_w = linear_vel / rear_wheel_radius_;
                out.rear_right_w = linear_vel / rear_wheel_radius_;

                applyRearCompressionBias(
                    linear_vel,
                    out.rear_left_w,
                    out.rear_right_w);

                return out;
            }

            const double x_front = wheel_base_;

            const double y_front_left = front_track_width_ / 2.0;
            const double y_front_right = -front_track_width_ / 2.0;

            const double y_rear_left = rear_track_width_ / 2.0;
            const double y_rear_right = -rear_track_width_ / 2.0;

            const double v_fl_x = linear_vel - effective_angular_vel * y_front_left;
            const double v_fr_x = linear_vel - effective_angular_vel * y_front_right;
            const double v_f_y = effective_angular_vel * x_front;

            out.front_left_steer = std::atan2(v_f_y, v_fl_x);
            out.front_right_steer = std::atan2(v_f_y, v_fr_x);

            double v_fl = std::sqrt(v_fl_x * v_fl_x + v_f_y * v_f_y);
            double v_fr = std::sqrt(v_fr_x * v_fr_x + v_f_y * v_f_y);

            normalizeSteerAndWheelSpeed(out.front_left_steer, v_fl);
            normalizeSteerAndWheelSpeed(out.front_right_steer, v_fr);

            const double v_rl = linear_vel - effective_angular_vel * y_rear_left;
            const double v_rr = linear_vel - effective_angular_vel * y_rear_right;

            out.front_left_w = v_fl / front_wheel_radius_;
            out.front_right_w = v_fr / front_wheel_radius_;

            out.rear_left_w = v_rl / rear_wheel_radius_;
            out.rear_right_w = v_rr / rear_wheel_radius_;

            applyRearCompressionBias(
                linear_vel,
                out.rear_left_w,
                out.rear_right_w);

            return out;
        }

    private:
        static void normalizeSteerAndWheelSpeed(
            double &steer,
            double &wheel_linear_speed)
        {
            constexpr double kPi = 3.14159265358979323846;
            constexpr double kHalfPi = kPi / 2.0;

            if (steer > kHalfPi)
            {
                steer -= kPi;
                wheel_linear_speed = -wheel_linear_speed;
            }
            else if (steer < -kHalfPi)
            {
                steer += kPi;
                wheel_linear_speed = -wheel_linear_speed;
            }
        }

        static void applyRearCompressionBias(
            double linear_vel,
            double &rear_left_w,
            double &rear_right_w)
        {
            constexpr double kCompressionBias = 0.10;
            constexpr double kCompressionMinLinearVel = 0.05;

            if (linear_vel > kCompressionMinLinearVel)
            {
                rear_left_w *= (1.0 + kCompressionBias);
                rear_right_w *= (1.0 + kCompressionBias);
            }
            else if (linear_vel < -kCompressionMinLinearVel)
            {
                rear_left_w *= (1.0 - kCompressionBias);
                rear_right_w *= (1.0 - kCompressionBias);
            }
        }

    private:
        double wheel_base_;
        double front_track_width_;
        double rear_track_width_;
        double front_wheel_radius_;
        double rear_wheel_radius_;
    };

} // namespace vehicle_kinematics