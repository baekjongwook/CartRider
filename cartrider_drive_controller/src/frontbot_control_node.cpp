// Frontbot Control Node
// 2026.04.15 백종욱

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>

#include "cartrider_rmd_sdk/msg/motor_command_array.hpp"
#include "cartrider_vesc_sdk/msg/motor_command_array.hpp"

#include "cartrider_drive_controller/differential_drive.hpp"
#include "cartrider_drive_controller/ackermann_drive.hpp"

class FrontbotControlNode : public rclcpp::Node
{
public:
    enum class DriveMode
    {
        DIFFERENTIAL,
        ACKERMANN
    };

    FrontbotControlNode()
        : Node("frontbot_control_node")
    {
        wheel_radius_ = this->declare_parameter<double>("wheel_radius");
        track_width_ = this->declare_parameter<double>("track_width");
        wheel_base_ = this->declare_parameter<double>("wheel_base");

        rmd_motor_ids_ =
            this->declare_parameter<std::vector<int64_t>>("rmd_motor_ids", std::vector<int64_t>{});
        vesc_motor_ids_ =
            this->declare_parameter<std::vector<int64_t>>("vesc_motor_ids", std::vector<int64_t>{});

        if (rmd_motor_ids_.size() != 2)
        {
            RCLCPP_FATAL(
                this->get_logger(),
                "Parameter 'rmd_motor_ids' must contain exactly 2 elements. Got %zu",
                rmd_motor_ids_.size());
            throw std::runtime_error("Invalid rmd_motor_ids size");
        }

        if (vesc_motor_ids_.size() != 2)
        {
            RCLCPP_FATAL(
                this->get_logger(),
                "Parameter 'vesc_motor_ids' must contain exactly 2 elements. Got %zu",
                vesc_motor_ids_.size());
            throw std::runtime_error("Invalid vesc_motor_ids size");
        }

        left_rmd_id_ = static_cast<int>(rmd_motor_ids_[0]);
        right_rmd_id_ = static_cast<int>(rmd_motor_ids_[1]);

        left_vesc_id_ = static_cast<int>(vesc_motor_ids_[0]);
        right_vesc_id_ = static_cast<int>(vesc_motor_ids_[1]);

        const std::string drive_mode_str =
            this->declare_parameter<std::string>("drive_mode", "differential");

        if (drive_mode_str == "differential")
        {
            drive_mode_ = DriveMode::DIFFERENTIAL;
        }
        else if (drive_mode_str == "ackermann")
        {
            drive_mode_ = DriveMode::ACKERMANN;
        }
        else
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Unknown drive_mode '%s'. Falling back to 'differential'.",
                drive_mode_str.c_str());
            drive_mode_ = DriveMode::DIFFERENTIAL;
        }

        diff_ = std::make_unique<vehicle_kinematics::DifferentialDrive>(
            wheel_radius_, track_width_);

        ackermann_ = std::make_unique<vehicle_kinematics::AckermannDrive>(
            wheel_base_, track_width_, wheel_radius_);

        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel",
            10,
            std::bind(&FrontbotControlNode::cmdCallback, this, std::placeholders::_1));

        cmd_joy_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel_joy",
            10,
            std::bind(&FrontbotControlNode::cmdJoyCallback, this, std::placeholders::_1));

        joy_sig_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "joy_control_sig",
            10,
            std::bind(&FrontbotControlNode::joySigCallback, this, std::placeholders::_1));

        docking_state_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "docking_state",
            10,
            std::bind(&FrontbotControlNode::dockingStateCallback, this, std::placeholders::_1));

        rmd_command_pub_ =
            this->create_publisher<cartrider_rmd_sdk::msg::MotorCommandArray>("rmd_command", 10);

        vesc_command_pub_ =
            this->create_publisher<cartrider_vesc_sdk::msg::MotorCommandArray>("vesc_command", 10);

        RCLCPP_INFO(
            this->get_logger(),
            "Frontbot Control Node Started. Input Mode: JOYSTICK, Drive Mode: %s",
            driveModeToString(drive_mode_).c_str());
    }

private:
    bool joy_mode_active_{true};
    DriveMode drive_mode_{DriveMode::DIFFERENTIAL};

    std::unique_ptr<vehicle_kinematics::DifferentialDrive> diff_;
    std::unique_ptr<vehicle_kinematics::AckermannDrive> ackermann_;

    double wheel_radius_{0.0};
    double track_width_{0.0};
    double wheel_base_{0.0};

    std::vector<int64_t> rmd_motor_ids_;
    std::vector<int64_t> vesc_motor_ids_;

    int left_rmd_id_{0};
    int right_rmd_id_{0};
    int left_vesc_id_{0};
    int right_vesc_id_{0};

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_joy_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr joy_sig_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr docking_state_sub_;

    rclcpp::Publisher<cartrider_rmd_sdk::msg::MotorCommandArray>::SharedPtr rmd_command_pub_;
    rclcpp::Publisher<cartrider_vesc_sdk::msg::MotorCommandArray>::SharedPtr vesc_command_pub_;

private:
    std::string driveModeToString(DriveMode mode) const
    {
        switch (mode)
        {
        case DriveMode::DIFFERENTIAL:
            return "DIFFERENTIAL";
        case DriveMode::ACKERMANN:
            return "ACKERMANN";
        default:
            return "UNKNOWN";
        }
    }

    void joySigCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        joy_mode_active_ = msg->data;

        if (joy_mode_active_)
        {
            RCLCPP_INFO(this->get_logger(), "Control Input Mode Switched: [ JOYSTICK ]");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Control Input Mode Switched: [ NAVIGATION ]");
        }
    }

    void dockingStateCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        const DriveMode new_mode =
            msg->data ? DriveMode::ACKERMANN : DriveMode::DIFFERENTIAL;

        if (new_mode == drive_mode_)
        {
            return;
        }

        drive_mode_ = new_mode;

        RCLCPP_INFO(
            this->get_logger(),
            "Drive Mode Switched by docking_state: %s",
            driveModeToString(drive_mode_).c_str());
    }

    void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (joy_mode_active_)
        {
            return;
        }
        handleCommand(msg, "NAV");
    }

    void cmdJoyCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (!joy_mode_active_)
        {
            return;
        }
        handleCommand(msg, "JOY");
    }

    void handleCommand(
        const geometry_msgs::msg::Twist::SharedPtr msg,
        const std::string &source)
    {
        switch (drive_mode_)
        {
        case DriveMode::DIFFERENTIAL:
            publishDifferentialCommand(msg, source);
            break;

        case DriveMode::ACKERMANN:
            publishAckermannCommand(msg, source);
            break;

        default:
            RCLCPP_ERROR(this->get_logger(), "Invalid drive mode.");
            break;
        }
    }

    void publishDifferentialCommand(
        const geometry_msgs::msg::Twist::SharedPtr msg,
        const std::string &source)
    {
        const auto output = diff_->compute(msg->linear.x, msg->angular.z);

        const double left_wheel_radps = output.left_w;
        const double right_wheel_radps = -output.right_w;

        RCLCPP_INFO(
            this->get_logger(),
            "[%s][DIFF] cmd_vel: v=%.3f [m/s] w=%.3f [rad/s] -> "
            "left_wheel=%.3f [rad/s], right_wheel=%.3f [rad/s]",
            source.c_str(),
            msg->linear.x,
            msg->angular.z,
            left_wheel_radps,
            right_wheel_radps);

        cartrider_rmd_sdk::msg::MotorCommandArray rmd_cmd;
        cartrider_rmd_sdk::msg::MotorCommand left_drive_cmd;
        cartrider_rmd_sdk::msg::MotorCommand right_drive_cmd;

        cartrider_vesc_sdk::msg::MotorCommandArray vesc_cmd;
        cartrider_vesc_sdk::msg::MotorCommand left_yaw_cmd;
        cartrider_vesc_sdk::msg::MotorCommand right_yaw_cmd;

        left_drive_cmd.id = left_rmd_id_;
        right_drive_cmd.id = right_rmd_id_;
        left_yaw_cmd.id = left_vesc_id_;
        right_yaw_cmd.id = right_vesc_id_;

        left_drive_cmd.target = left_wheel_radps;
        right_drive_cmd.target = right_wheel_radps;

        left_yaw_cmd.target = 0.0;
        right_yaw_cmd.target = 0.0;

        rmd_cmd.commands.push_back(left_drive_cmd);
        rmd_cmd.commands.push_back(right_drive_cmd);
        vesc_cmd.commands.push_back(left_yaw_cmd);
        vesc_cmd.commands.push_back(right_yaw_cmd);

        rmd_command_pub_->publish(rmd_cmd);
        vesc_command_pub_->publish(vesc_cmd);
    }

    void publishAckermannCommand(
        const geometry_msgs::msg::Twist::SharedPtr msg,
        const std::string &source)
    {
        const auto output = ackermann_->compute(msg->linear.x, msg->angular.z);

        const double left_steer_rad = output.left_steer;
        const double right_steer_rad = output.right_steer;

        const double left_wheel_radps = output.left_w;
        const double right_wheel_radps = -output.right_w;

        RCLCPP_INFO(
            this->get_logger(),
            "[%s][ACK] cmd_vel: v=%.3f [m/s] w=%.3f [rad/s] -> "
            "left_steer=%.3f [rad], right_steer=%.3f [rad], "
            "left_wheel=%.3f [rad/s], right_wheel=%.3f [rad/s]",
            source.c_str(),
            msg->linear.x,
            msg->angular.z,
            left_steer_rad,
            right_steer_rad,
            left_wheel_radps,
            right_wheel_radps);

        cartrider_rmd_sdk::msg::MotorCommandArray rmd_cmd;
        cartrider_rmd_sdk::msg::MotorCommand left_drive_cmd;
        cartrider_rmd_sdk::msg::MotorCommand right_drive_cmd;

        cartrider_vesc_sdk::msg::MotorCommandArray vesc_cmd;
        cartrider_vesc_sdk::msg::MotorCommand left_yaw_cmd;
        cartrider_vesc_sdk::msg::MotorCommand right_yaw_cmd;

        left_drive_cmd.id = left_rmd_id_;
        right_drive_cmd.id = right_rmd_id_;
        left_yaw_cmd.id = left_vesc_id_;
        right_yaw_cmd.id = right_vesc_id_;

        left_drive_cmd.target = left_wheel_radps;
        right_drive_cmd.target = right_wheel_radps;

        left_yaw_cmd.target = left_steer_rad;
        right_yaw_cmd.target = right_steer_rad;

        rmd_cmd.commands.push_back(left_drive_cmd);
        rmd_cmd.commands.push_back(right_drive_cmd);
        vesc_cmd.commands.push_back(left_yaw_cmd);
        vesc_cmd.commands.push_back(right_yaw_cmd);

        rmd_command_pub_->publish(rmd_cmd);
        vesc_command_pub_->publish(vesc_cmd);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrontbotControlNode>());
    rclcpp::shutdown();
    return 0;
}