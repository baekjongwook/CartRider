// Teleop_joystick
// 2026.03.16 백종욱

// button 1: /front/home true
// button 2: /front/cart_docking true
// button 3: /front/robot_docking true
// button 4: /cart_count 0
// button 5: /cart_count 2
// axes[scenario_sequence_axis_index] ==  1: /start_patrol_mission 1
// axes[scenario_sequence_axis_index] == -1: /start_patrol_mission 2
// button 9: /gripper_toggle toggle

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/int32.hpp>

#include <iostream>
#include <string>
#include <cmath>
#include <cstdint>

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

        gripper_toggle_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/gripper_toggle",
            10);

        cart_count_pub_ = this->create_publisher<std_msgs::msg::UInt16>(
            "/cart_count",
            10);

        start_patrol_mission_pub_ = this->create_publisher<std_msgs::msg::Int32>(
            "/start_patrol_mission",
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

        cart_count_zero_button_index_ =
            this->declare_parameter<int>("cart_count_zero_button_index", 4);
        cart_count_two_button_index_ =
            this->declare_parameter<int>("cart_count_two_button_index", 5);

        scenario_sequence_axis_index_ =
            this->declare_parameter<int>("scenario_sequence_axis_index", 6);

        gripper_toggle_button_index_ =
            this->declare_parameter<int>("gripper_toggle_button_index", 9);

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

        RCLCPP_INFO(
            this->get_logger(),
            "Cart count buttons: zero=%d two=%d",
            cart_count_zero_button_index_,
            cart_count_two_button_index_);

        RCLCPP_INFO(
            this->get_logger(),
            "Scenario sequence axis: axis[%d]",
            scenario_sequence_axis_index_);

        RCLCPP_INFO(
            this->get_logger(),
            "Dynamixel gripper toggle button: %d",
            gripper_toggle_button_index_);
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr rearbot_cmd_joy_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr frontbot_cmd_joy_pub_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr rear_joy_sig_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr front_joy_sig_pub_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr home_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr cart_docking_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr robot_docking_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripper_toggle_pub_;

    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr cart_count_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr start_patrol_mission_pub_;

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

    bool cart_count_zero_btn_once_{true};
    bool cart_count_two_btn_once_{true};

    bool scenario_sequence_axis_once_{true};

    bool gripper_toggle_btn_once_{true};
    bool gripper_toggle_state_{false};

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

    int cart_count_zero_button_index_{4};
    int cart_count_two_button_index_{5};

    int scenario_sequence_axis_index_{6};

    int gripper_toggle_button_index_{9};

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

    double getAxis(
        const sensor_msgs::msg::Joy::SharedPtr msg,
        int index) const
    {
        if (index < 0 || index >= static_cast<int>(msg->axes.size()))
        {
            return 0.0;
        }

        return msg->axes[index];
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
        publishGripperToggleState();
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

    void publishCartCount(uint16_t count, const std::string &reason)
    {
        std_msgs::msg::UInt16 msg;
        msg.data = count;
        cart_count_pub_->publish(msg);

        RCLCPP_INFO(
            this->get_logger(),
            "Cart count published: %u (%s)",
            static_cast<unsigned int>(count),
            reason.c_str());
    }

    void publishStartPatrolMission(int32_t mission_id, const std::string &reason)
    {
        std_msgs::msg::Int32 msg;
        msg.data = mission_id;
        start_patrol_mission_pub_->publish(msg);

        RCLCPP_INFO(
            this->get_logger(),
            "Start patrol mission published: %d (%s)",
            mission_id,
            reason.c_str());
    }

    void publishGripperToggleState()
    {
        std_msgs::msg::Bool msg;
        msg.data = gripper_toggle_state_;
        gripper_toggle_pub_->publish(msg);

        RCLCPP_INFO(
            this->get_logger(),
            "Dynamixel gripper toggle published: %s",
            gripper_toggle_state_ ? "true(GRIP)" : "false(RELEASE)");
    }

    void dockingStateCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        const bool prev = docking_state_;
        docking_state_ = msg->data;

        if (docking_state_)
        {
            control_mode_ = ControlMode::MULTIBOT_ACKERMANN;
        }
        else
        {
            if (control_mode_ == ControlMode::MULTIBOT_ACKERMANN)
            {
                control_mode_ = ControlMode::REARBOT_INDEPENDENT;
            }
        }

        cmd_vel_was_zero_ = true;

        if (prev != docking_state_)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "docking_state=%s -> joystick mode=%s",
                docking_state_ ? "true" : "false",
                controlModeToString(control_mode_).c_str());
        }
    }

    void nextControlMode()
    {
        if (docking_state_)
        {
            control_mode_ = ControlMode::MULTIBOT_ACKERMANN;
            cmd_vel_was_zero_ = true;

            RCLCPP_INFO(
                this->get_logger(),
                "Docked state. Joystick mode fixed to [%s].",
                controlModeToString(control_mode_).c_str());
            return;
        }

        switch (control_mode_)
        {
        case ControlMode::REARBOT_INDEPENDENT:
            control_mode_ = ControlMode::FRONTBOT_INDEPENDENT;
            break;

        case ControlMode::FRONTBOT_INDEPENDENT:
            control_mode_ = ControlMode::REARBOT_INDEPENDENT;
            break;

        case ControlMode::MULTIBOT_ACKERMANN:
        default:
            control_mode_ = ControlMode::REARBOT_INDEPENDENT;
            break;
        }

        cmd_vel_was_zero_ = true;

        RCLCPP_INFO(
            this->get_logger(),
            "Joystick Control Mode Switched: [%s], docking_state=false",
            controlModeToString(control_mode_).c_str());
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

    void handleCartCountButtons(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        const int button_zero = getButton(msg, cart_count_zero_button_index_);
        const int button_two = getButton(msg, cart_count_two_button_index_);

        if (button_zero == 1 && cart_count_zero_btn_once_)
        {
            publishCartCount(0, "button_zero");
            cart_count_zero_btn_once_ = false;
        }
        else if (button_zero == 0)
        {
            cart_count_zero_btn_once_ = true;
        }

        if (button_two == 1 && cart_count_two_btn_once_)
        {
            publishCartCount(2, "button_two");
            cart_count_two_btn_once_ = false;
        }
        else if (button_two == 0)
        {
            cart_count_two_btn_once_ = true;
        }
    }

    void handleScenarioSequenceAxis(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        const double axis_value = getAxis(msg, scenario_sequence_axis_index_);

        if (axis_value > 0.9 && scenario_sequence_axis_once_)
        {
            publishStartPatrolMission(1, "scenario_sequence_1");
            scenario_sequence_axis_once_ = false;
        }
        else if (axis_value < -0.9 && scenario_sequence_axis_once_)
        {
            publishStartPatrolMission(2, "scenario_sequence_2");
            scenario_sequence_axis_once_ = false;
        }
        else if (std::abs(axis_value) < 0.1)
        {
            scenario_sequence_axis_once_ = true;
        }
    }

    void handleGripperToggleButton(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        const int button_gripper = getButton(msg, gripper_toggle_button_index_);

        if (button_gripper == 1 && gripper_toggle_btn_once_)
        {
            gripper_toggle_state_ = !gripper_toggle_state_;
            publishGripperToggleState();

            gripper_toggle_btn_once_ = false;
        }
        else if (button_gripper == 0)
        {
            gripper_toggle_btn_once_ = true;
        }
    }

    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        geometry_msgs::msg::Twist cmd;
        bool is_zero = false;

        handleMightyZapButtons(msg);
        handleCartCountButtons(msg);
        handleScenarioSequenceAxis(msg);
        handleGripperToggleButton(msg);

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
                  << "  DockingState: " << (docking_state_ ? "true" : "false")
                  << "  Gripper: " << (gripper_toggle_state_ ? "GRIP" : "RELEASE")
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