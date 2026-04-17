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
    enum class RobotType
    {
        REARBOT,
        FRONTBOT
    };

    Teleop() : Node("teleop_joystick")
    {
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "cmd_vel_joy",
            10);

        joy_sig_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "joy_control_sig",
            10);

        docking_state_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "docking_state",
            10);

        sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy",
            10,
            std::bind(&Teleop::joyCallback, this, std::placeholders::_1));

        robot_type_str_ =
            this->declare_parameter<std::string>("robot_type", "rearbot");

        rearbot_max_lin_ =
            this->declare_parameter<double>("rearbot_max_lin");
        rearbot_max_ang_ =
            this->declare_parameter<double>("rearbot_max_ang");

        frontbot_diff_max_lin_ =
            this->declare_parameter<double>("frontbot_diff_max_lin");
        frontbot_diff_max_ang_ =
            this->declare_parameter<double>("frontbot_diff_max_ang");

        frontbot_ack_max_lin_ =
            this->declare_parameter<double>("frontbot_ack_max_lin");
        frontbot_ack_max_ang_ =
            this->declare_parameter<double>("frontbot_ack_max_ang");

        linear_axis_index_ =
            this->declare_parameter<int>("linear_axis_index");
        angular_axis_index_ =
            this->declare_parameter<int>("angular_axis_index");

        x_button_index_ =
            this->declare_parameter<int>("x_button_index");
        ps_button_index_ =
            this->declare_parameter<int>("ps_button_index");

        if (robot_type_str_ == "rearbot")
        {
            robot_type_ = RobotType::REARBOT;
        }
        else if (robot_type_str_ == "frontbot")
        {
            robot_type_ = RobotType::FRONTBOT;
        }
        else
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Unknown robot_type '%s'. Falling back to 'rearbot'.",
                robot_type_str_.c_str());
            robot_type_ = RobotType::REARBOT;
        }

        publishInitialStates();

        std::cout << "Joystick Teleop Started\n";

        RCLCPP_INFO(
            this->get_logger(),
            "Robot type: %s | Initial frontbot mode: %s",
            robotTypeToString(robot_type_).c_str(),
            docking_state_ ? "ACKERMANN" : "DIFFERENTIAL");
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr joy_sig_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr docking_state_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;

    bool cmd_vel_was_zero_ = true;
    bool joy_mode_active_ = true;

    bool docking_state_ = false; // false=differential, true=ackermann

    bool x_btn_once_ = true;
    bool ps_btn_once_ = true;

    RobotType robot_type_{RobotType::REARBOT};
    std::string robot_type_str_{"rearbot"};

    double rearbot_max_lin_{0.5};
    double rearbot_max_ang_{0.2};

    double frontbot_diff_max_lin_{0.5};
    double frontbot_diff_max_ang_{0.2};

    double frontbot_ack_max_lin_{0.3};
    double frontbot_ack_max_ang_{0.15};

    int linear_axis_index_{1};
    int angular_axis_index_{3};

    int x_button_index_{0};
    int ps_button_index_{10};

private:
    std::string robotTypeToString(RobotType type) const
    {
        switch (type)
        {
        case RobotType::REARBOT:
            return "REARBOT";
        case RobotType::FRONTBOT:
            return "FRONTBOT";
        default:
            return "UNKNOWN";
        }
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
        msg.data = docking_state_;
        docking_state_pub_->publish(msg);
    }

    void publishInitialStates()
    {
        publishJoyMode();
        publishDockingState();
    }

    void getCurrentGains(double &max_lin, double &max_ang) const
    {
        if (robot_type_ == RobotType::REARBOT)
        {
            max_lin = rearbot_max_lin_;
            max_ang = rearbot_max_ang_;
            return;
        }

        if (!docking_state_) // differential
        {
            max_lin = frontbot_diff_max_lin_;
            max_ang = frontbot_diff_max_ang_;
        }
        else // ackermann
        {
            max_lin = frontbot_ack_max_lin_;
            max_ang = frontbot_ack_max_ang_;
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

            if (joy_mode_active_)
            {
                RCLCPP_INFO(this->get_logger(), "Control Input Mode Switched: [ JOYSTICK ]");
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Control Input Mode Switched: [ NAVIGATION ]");
            }

            x_btn_once_ = false;
        }
        else if (button_x == 0)
        {
            x_btn_once_ = true;
        }

        if (robot_type_ == RobotType::FRONTBOT)
        {
            if (button_ps == 1 && ps_btn_once_)
            {
                docking_state_ = !docking_state_;
                publishDockingState();

                RCLCPP_INFO(
                    this->get_logger(),
                    "Frontbot Drive Mode Switched: %s",
                    docking_state_ ? "ACKERMANN" : "DIFFERENTIAL");

                ps_btn_once_ = false;
            }
            else if (button_ps == 0)
            {
                ps_btn_once_ = true;
            }
        }
        else
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

            if (std::abs(cmd.linear.x) < 1e-9 && std::abs(cmd.angular.z) < 1e-9)
            {
                is_zero = true;
            }
        }

        if (is_zero && cmd_vel_was_zero_)
        {
            return;
        }

        pub_->publish(cmd);
        cmd_vel_was_zero_ = is_zero;

        std::cout << "\rRobot: " << robotTypeToString(robot_type_)
                  << "  Mode: "
                  << (robot_type_ == RobotType::FRONTBOT
                          ? (docking_state_ ? "ACKERMANN" : "DIFFERENTIAL")
                          : "REARBOT")
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