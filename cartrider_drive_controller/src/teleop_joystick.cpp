// Teleop_joystick
// 2026.03.05 백종욱

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <iostream>
#include <algorithm>

#define LIN_STEP 0.01
#define ANG_STEP 0.05

#define MAX_LIN 2.0
#define MAX_ANG 3.0

class Teleop : public rclcpp::Node
{
public:
    Teleop() : Node("teleop_joystick")
    {
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy",
            10,
            std::bind(&Teleop::joyCallback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&Teleop::update, this));

        std::cout << "Joystick Teleop Started\n";
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double target_linear_ = 0.0;
    double target_angular_ = 0.0;
    double control_linear_ = 0.0;
    double control_angular_ = 0.0;

    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if (msg->buttons[0])
        {
            target_linear_ = 0.0;
            target_angular_ = 0.0;
            control_linear_ = 0.0;
            control_angular_ = 0.0;
            return;
        }

        double linear_axis  = msg->axes[1]; 
        double angular_axis = msg->axes[3]; 

        target_linear_  = linear_axis * MAX_LIN;
        target_angular_ = angular_axis * MAX_ANG;
    }

    void update()
    {
        control_linear_ = makeProfile(control_linear_, target_linear_, LIN_STEP/2.0);

        control_angular_ = makeProfile(control_angular_, target_angular_, ANG_STEP/2.0);

        publish();
    }

    void publish()
    {
        geometry_msgs::msg::Twist msg;

        msg.linear.x = control_linear_;
        msg.angular.z = control_angular_;

        pub_->publish(msg);

        std::cout << "\rLinear: " << control_linear_
                << "  Angular: " << control_angular_
                << "   " << std::flush;
    }

    double makeProfile(double output, double input, double slop)
    {
        if (input > output)
            output = std::min(input, output + slop);
        else if (input < output)
            output = std::max(input, output - slop);

        return output;
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Teleop>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}