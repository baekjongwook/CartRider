// Teleop_joystick
// 2026.03.16 백종욱

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>

#include <iostream>
#include <string>
#include <cmath>

class Teleop : public rclcpp::Node
{
public:
    enum class ControlMode
    {
        REARBOT_INDEPENDENT = 0,
        FRONTBOT_INDEPENDENT = 1,
        MULTIBOT_ACKERMANN = 2
    };

    Teleop() : Node("teleop_joystick")
    {
        rearbot_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel",
            10);

        frontbot_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/front/cmd_vel",
            10);

        joy_sig_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/joy_control_sig",
            10);

        docking_state_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/docking_state",
            10);

        sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy",
            10,
            std::bind(&Teleop::joyCallback, this, std::placeholders::_1));

        rearbot_cmd_vel_max_lin_ =
            this->declare_parameter<double>("rearbot_cmd_vel_max_lin");
        rearbot_cmd_vel_max_ang_ =
            this->declare_parameter<double>("rearbot_cmd_vel_max_ang");

        frontbot_cmd_vel_max_lin_ =
            this->declare_parameter<double>("frontbot_cmd_vel_max_lin");
        frontbot_cmd_vel_max_ang_ =
            this->declare_parameter<double>("frontbot_cmd_vel_max_ang");

        multibot_cmd_vel_max_lin_ =
            this->declare_parameter<double>("multibot_cmd_vel_max_lin");
        multibot_cmd_vel_max_ang_ =
            this->declare_parameter<double>("multibot_cmd_vel_max_ang");

        linear_axis_index_ =
            this->declare_parameter<int>("linear_axis_index");
        angular_axis_index_ =
            this->declare_parameter<int>("angular_axis_index");

        x_button_index_ =
            this->declare_parameter<int>("x_button_index");
        ps_button_index_ =
            this->declare_parameter<int>("ps_button_index");

        publishInitialStates();

        RCLCPP_INFO(
            this->get_logger(),
            "Joystick Teleop Started. Initial Mode: %s",
            controlModeToString(control_mode_).c_str());
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr rearbot_cmd_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr frontbot_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr joy_sig_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr docking_state_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;

    bool cmd_vel_was_zero_{true};
    bool joy_mode_active_{true};

    bool x_btn_once_{true};
    bool ps_btn_once_{true};

    ControlMode control_mode_{ControlMode::REARBOT_INDEPENDENT};

    double rearbot_cmd_vel_max_lin_;
    double rearbot_cmd_vel_max_ang_;

    double frontbot_cmd_vel_max_lin_;
    double frontbot_cmd_vel_max_ang_;

    double multibot_cmd_vel_max_lin_;
    double multibot_cmd_vel_max_ang_;

    int linear_axis_index_{1};
    int angular_axis_index_{3};

    int x_button_index_{0};
    int ps_button_index_{10};

private:
    std::string controlModeToString(ControlMode mode) const
    {
        switch (mode)
        {
        case ControlMode::REARBOT_INDEPENDENT:
            return "REARBOT_INDEPENDENT";
        case ControlMode::FRONTBOT_INDEPENDENT:
            return "FRONTBOT_INDEPENDENT";
        case ControlMode::MULTIBOT_ACKERMANN:
            return "MULTIBOT_ACKERMANN";
        default:
            return "UNKNOWN";
        }
    }

    bool isDockedMode() const
    {
        return control_mode_ == ControlMode::MULTIBOT_ACKERMANN;
    }

    void publishJoyMode()
    {
        std_msgs::msg::Bool msg;
        msg.data = joy_mode_active_;
        joy_sig_pub_->publish(msg);
    }

    void publishDockingState()
    {
        std_msgs::msg::Bool msg;
        msg.data = isDockedMode();
        docking_state_pub_->publish(msg);
    }

    void publishInitialStates()
    {
        publishJoyMode();
        publishDockingState();
    }

    void nextControlMode()
    {
        switch (control_mode_)
        {
        case ControlMode::REARBOT_INDEPENDENT:
            control_mode_ = ControlMode::FRONTBOT_INDEPENDENT;
            break;

        case ControlMode::FRONTBOT_INDEPENDENT:
            control_mode_ = ControlMode::MULTIBOT_ACKERMANN;
            break;

        case ControlMode::MULTIBOT_ACKERMANN:
            control_mode_ = ControlMode::REARBOT_INDEPENDENT;
            break;

        default:
            control_mode_ = ControlMode::REARBOT_INDEPENDENT;
            break;
        }

        cmd_vel_was_zero_ = true;
        publishDockingState();

        RCLCPP_INFO(
            this->get_logger(),
            "Joystick Control Mode Switched: [%s], docking_state=%s",
            controlModeToString(control_mode_).c_str(),
            isDockedMode() ? "true" : "false");
    }

    void getCurrentGains(double &max_lin, double &max_ang) const
    {
        switch (control_mode_)
        {
        case ControlMode::REARBOT_INDEPENDENT:
            max_lin = rearbot_cmd_vel_max_lin_;
            max_ang = rearbot_cmd_vel_max_ang_;
            break;

        case ControlMode::FRONTBOT_INDEPENDENT:
            max_lin = frontbot_cmd_vel_max_lin_;
            max_ang = frontbot_cmd_vel_max_ang_;
            break;

        case ControlMode::MULTIBOT_ACKERMANN:
            max_lin = multibot_cmd_vel_max_lin_;
            max_ang = multibot_cmd_vel_max_ang_;
            break;

        default:
            max_lin = 0.0;
            max_ang = 0.0;
            break;
        }
    }

    void publishCommand(const geometry_msgs::msg::Twist &cmd)
    {
        switch (control_mode_)
        {
        case ControlMode::REARBOT_INDEPENDENT:
            rearbot_cmd_pub_->publish(cmd); // /cmd_vel
            break;

        case ControlMode::FRONTBOT_INDEPENDENT:
            frontbot_cmd_pub_->publish(cmd); // /front/cmd_vel
            break;

        case ControlMode::MULTIBOT_ACKERMANN:
            rearbot_cmd_pub_->publish(cmd); // /cmd_vel
            break;

        default:
            break;
        }
    }

    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        geometry_msgs::msg::Twist cmd;
        bool is_zero = false;

        const int button_x =
            (x_button_index_ >= 0 && x_button_index_ < static_cast<int>(msg->buttons.size()))
                ? msg->buttons[x_button_index_]
                : 0;

        const int button_ps =
            (ps_button_index_ >= 0 && ps_button_index_ < static_cast<int>(msg->buttons.size()))
                ? msg->buttons[ps_button_index_]
                : 0;

        if (button_x == 1 && x_btn_once_)
        {
            joy_mode_active_ = !joy_mode_active_;
            publishJoyMode();

            RCLCPP_INFO(
                this->get_logger(),
                "Control Input Mode Switched: [%s]",
                joy_mode_active_ ? "JOYSTICK" : "NAVIGATION");

            x_btn_once_ = false;
        }
        else if (button_x == 0)
        {
            x_btn_once_ = true;
        }

        if (button_ps == 1 && ps_btn_once_)
        {
            nextControlMode();
            ps_btn_once_ = false;
        }
        else if (button_ps == 0)
        {
            ps_btn_once_ = true;
        }

        if (button_x)
        {
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
            is_zero = true;
        }
        else
        {
            double max_lin = 0.0;
            double max_ang = 0.0;
            getCurrentGains(max_lin, max_ang);

            double linear_axis = 0.0;
            double angular_axis = 0.0;

            if (linear_axis_index_ >= 0 &&
                linear_axis_index_ < static_cast<int>(msg->axes.size()))
            {
                linear_axis = msg->axes[linear_axis_index_];
            }

            if (angular_axis_index_ >= 0 &&
                angular_axis_index_ < static_cast<int>(msg->axes.size()))
            {
                angular_axis = msg->axes[angular_axis_index_];
            }

            cmd.linear.x = linear_axis * max_lin;
            cmd.angular.z = angular_axis * max_ang;

            if (std::abs(cmd.linear.x) < 1e-9 &&
                std::abs(cmd.angular.z) < 1e-9)
            {
                is_zero = true;
            }
        }

        if (is_zero && cmd_vel_was_zero_)
        {
            return;
        }

        publishCommand(cmd);
        cmd_vel_was_zero_ = is_zero;

        std::cout << "\rMode: " << controlModeToString(control_mode_)
                  << "  Docked: " << (isDockedMode() ? "true" : "false")
                  << "  Linear: " << cmd.linear.x
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