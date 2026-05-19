// Teleop_joystick
// 2026.03.16 백종욱
// Modified: mightyZAP action buttons added
// button 1: home
// button 2: cart_docking
// button 3: robot_docking

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
        rearbot_cmd_joy_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel_joy",
            10);

        frontbot_cmd_joy_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/front/cmd_vel_joy",
            10);

        rear_joy_sig_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/joy_control_sig",
            10);

        front_joy_sig_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/front/joy_control_sig",
            10);

        home_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/front/home",
            10);

        cart_docking_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/front/cart_docking",
            10);

        robot_docking_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/front/robot_docking",
            10);

        docking_state_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/docking_state",
            10,
            std::bind(&Teleop::dockingStateCallback, this, std::placeholders::_1));

        sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy",
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

        home_button_index_ =
            this->declare_parameter<int>("home_button_index", 1);
        cart_docking_button_index_ =
            this->declare_parameter<int>("cart_docking_button_index", 2);
        robot_docking_button_index_ =
            this->declare_parameter<int>("robot_docking_button_index", 3);

        publishInitialStates();

        RCLCPP_INFO(
            this->get_logger(),
            "Joystick Teleop Started. Initial Mode: %s, Joy Input Mode: %s",
            controlModeToString(control_mode_).c_str(),
            joy_mode_active_ ? "JOYSTICK" : "NAVIGATION");

        RCLCPP_INFO(
            this->get_logger(),
            "mightyZAP buttons: home=%d cart_docking=%d robot_docking=%d",
            home_button_index_,
            cart_docking_button_index_,
            robot_docking_button_index_);
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr rearbot_cmd_joy_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr frontbot_cmd_joy_pub_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr rear_joy_sig_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr front_joy_sig_pub_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr home_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr cart_docking_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr robot_docking_pub_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr docking_state_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;

    bool cmd_vel_was_zero_{true};
    bool joy_mode_active_{true};
    bool docking_state_{false};

    bool x_btn_once_{true};
    bool ps_btn_once_{true};

    bool home_btn_once_{true};
    bool cart_docking_btn_once_{true};
    bool robot_docking_btn_once_{true};

    ControlMode control_mode_{ControlMode::REARBOT_INDEPENDENT};

    double rearbot_cmd_vel_max_lin_{0.0};
    double rearbot_cmd_vel_max_ang_{0.0};

    double frontbot_cmd_vel_max_lin_{0.0};
    double frontbot_cmd_vel_max_ang_{0.0};

    double multibot_cmd_vel_max_lin_{0.0};
    double multibot_cmd_vel_max_ang_{0.0};

    int linear_axis_index_{1};
    int angular_axis_index_{3};

    int x_button_index_{0};
    int ps_button_index_{10};

    int home_button_index_{1};
    int cart_docking_button_index_{2};
    int robot_docking_button_index_{3};

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

    bool isDifferentialMode() const
    {
        return control_mode_ == ControlMode::REARBOT_INDEPENDENT ||
               control_mode_ == ControlMode::FRONTBOT_INDEPENDENT;
    }

    int getButton(
        const sensor_msgs::msg::Joy::SharedPtr msg,
        int index) const
    {
        if (index < 0 || index >= static_cast<int>(msg->buttons.size()))
        {
            return 0;
        }

        return msg->buttons[index];
    }

    void publishJoyMode()
    {
        std_msgs::msg::Bool msg;
        msg.data = joy_mode_active_;

        rear_joy_sig_pub_->publish(msg);
        front_joy_sig_pub_->publish(msg);
    }

    void publishInitialStates()
    {
        publishJoyMode();
    }

    void publishBoolCommand(
        const rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr &pub,
        const std::string &name)
    {
        std_msgs::msg::Bool msg;
        msg.data = true;
        pub->publish(msg);

        RCLCPP_INFO(
            this->get_logger(),
            "mightyZAP manual command published: %s",
            name.c_str());
    }

    void dockingStateCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        docking_state_ = msg->data;

        RCLCPP_INFO(
            this->get_logger(),
            "docking_state updated from system: %s",
            docking_state_ ? "true" : "false");
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

        RCLCPP_INFO(
            this->get_logger(),
            "Joystick Control Mode Switched: [%s], system docking_state=%s",
            controlModeToString(control_mode_).c_str(),
            docking_state_ ? "true" : "false");
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
            rearbot_cmd_joy_pub_->publish(cmd);
            break;

        case ControlMode::FRONTBOT_INDEPENDENT:
            frontbot_cmd_joy_pub_->publish(cmd);
            break;

        case ControlMode::MULTIBOT_ACKERMANN:
            rearbot_cmd_joy_pub_->publish(cmd);
            break;

        default:
            break;
        }
    }

    void handleMightyZapButtons(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        const int button_home = getButton(msg, home_button_index_);
        const int button_cart_docking = getButton(msg, cart_docking_button_index_);
        const int button_robot_docking = getButton(msg, robot_docking_button_index_);

        if (button_home == 1 && home_btn_once_)
        {
            publishBoolCommand(home_pub_, "home");
            home_btn_once_ = false;
        }
        else if (button_home == 0)
        {
            home_btn_once_ = true;
        }

        if (button_cart_docking == 1 && cart_docking_btn_once_)
        {
            publishBoolCommand(cart_docking_pub_, "cart_docking");
            cart_docking_btn_once_ = false;
        }
        else if (button_cart_docking == 0)
        {
            cart_docking_btn_once_ = true;
        }

        if (button_robot_docking == 1 && robot_docking_btn_once_)
        {
            publishBoolCommand(robot_docking_pub_, "robot_docking");
            robot_docking_btn_once_ = false;
        }
        else if (button_robot_docking == 0)
        {
            robot_docking_btn_once_ = true;
        }
    }

    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        geometry_msgs::msg::Twist cmd;
        bool is_zero = false;

        handleMightyZapButtons(msg);

        const int button_x = getButton(msg, x_button_index_);
        const int button_ps = getButton(msg, ps_button_index_);

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

            if (isDifferentialMode() && cmd.linear.x < 0.0)
            {
                cmd.angular.z = -cmd.angular.z;
            }

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
                  << "  JoyInput: " << (joy_mode_active_ ? "JOYSTICK" : "NAVIGATION")
                  << "  SystemDockingState: " << (docking_state_ ? "true" : "false")
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