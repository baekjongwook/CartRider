// Teleop_joystick
// 2026.03.16 백종욱

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <iostream>

#define MAX_LIN 2.0
#define MAX_ANG 3.0

class Teleop : public rclcpp::Node
{
public:
    Teleop() : Node("teleop_joystick")
    {
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "cmd_vel", 
            10);

        sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy",
            10,
            std::bind(&Teleop::joyCallback, this, std::placeholders::_1));

        std::cout << "Joystick Teleop Started\n";
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;

    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        geometry_msgs::msg::Twist cmd;

        if (msg->buttons[0])
        {
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
        }
        else
        {
            double linear_axis  = msg->axes[1]; 
            double angular_axis = msg->axes[3];

            cmd.linear.x = linear_axis * MAX_LIN;
            cmd.angular.z = angular_axis * MAX_ANG;
        }

        pub_->publish(cmd);

        std::cout << "\rLinear: " << cmd.linear.x
                  << "  Angular: " << cmd.angular.z
                  << "   " << std::flush;
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