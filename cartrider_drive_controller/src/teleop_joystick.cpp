// Teleop_joystick
// 2026.03.16 백종욱

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>

#include <iostream>

#define MAX_LIN 2.0
#define MAX_ANG 0.5

class Teleop : public rclcpp::Node
{
public:
    Teleop() : Node("teleop_joystick")
    {
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "cmd_vel_joy",
            10);

        joy_sig_pub = this->create_publisher<std_msgs::msg::Bool>(
            "joy_control_sig",
            10);

        docking_state_pub = this->create_publisher<std_msgs::msg::Bool>(
            "docking_state",
            10);

        sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy",
            10,
            std::bind(&Teleop::joyCallback, this, std::placeholders::_1));

        std::cout << "Joystick Teleop Started\n";
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr joy_sig_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr docking_state_pub;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;

    bool cmd_vel_was_zero = true;
    bool joy_mode_trig = true;
    bool docking_state = false;

    bool x_btn_once = true;
    bool ps_btn_once = true;

    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        geometry_msgs::msg::Twist cmd;
        std_msgs::msg::Bool cmd_mode;
        std_msgs::msg::Bool docking_msg;
        bool is_zero = false;

        int buttonX = msg->buttons[0];
        int buttonPS = msg->buttons[10];

        if (buttonX)
        {
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
            is_zero = true;
        }
        else
        {
            double linear_axis = msg->axes[1];
            double angular_axis = msg->axes[3];

            cmd.linear.x = linear_axis * MAX_LIN;
            cmd.angular.z = angular_axis * MAX_ANG;

            if (cmd.linear.x == 0.0 && cmd.angular.z == 0.0)
            {
                is_zero = true;
            }
        }

        if (buttonX == 1 && x_btn_once == true)
        {
            joy_mode_trig = !joy_mode_trig;
            cmd_mode.data = joy_mode_trig;
            joy_sig_pub->publish(cmd_mode);

            x_btn_once = false;
        }
        else if (buttonX == 0)
        {
            x_btn_once = true;
        }

        if (buttonPS == 1 && ps_btn_once == true)
        {
            docking_state = !docking_state;
            docking_msg.data = docking_state;
            docking_state_pub->publish(docking_msg);

            ps_btn_once = false;
        }
        else if (buttonPS == 0)
        {
            ps_btn_once = true;
        }

        if (is_zero && cmd_vel_was_zero)
        {
            return;
        }

        pub_->publish(cmd);

        cmd_vel_was_zero = is_zero;

        std::cout << "\rLinear: " << cmd.linear.x
                  << "  Angular: " << cmd.angular.z
                  << "   " << std::flush;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Teleop>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}