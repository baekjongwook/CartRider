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
        front_wheel_radius_ = this->declare_parameter<double>("front_wheel_radius");
        rear_wheel_radius_ = this->declare_parameter<double>("rear_wheel_radius");
        front_track_width_ = this->declare_parameter<double>("front_track_width");
        rear_track_width_ = this->declare_parameter<double>("rear_track_width");
        wheel_base_ = this->declare_parameter<double>("wheel_base");

        front_rmd_motor_ids_ =
            this->declare_parameter<std::vector<int64_t>>("front_rmd_motor_ids", std::vector<int64_t>{});
        front_vesc_motor_ids_ =
            this->declare_parameter<std::vector<int64_t>>("front_vesc_motor_ids", std::vector<int64_t>{});
        rear_rmd_motor_ids_ =
            this->declare_parameter<std::vector<int64_t>>("rear_rmd_motor_ids", std::vector<int64_t>{});

        if (front_rmd_motor_ids_.size() != 2)
        {
            RCLCPP_FATAL(this->get_logger(), "front_rmd_motor_ids must contain exactly 2 elements.");
            throw std::runtime_error("Invalid front_rmd_motor_ids size");
        }

        if (front_vesc_motor_ids_.size() != 2)
        {
            RCLCPP_FATAL(this->get_logger(), "front_vesc_motor_ids must contain exactly 2 elements.");
            throw std::runtime_error("Invalid front_vesc_motor_ids size");
        }

        if (rear_rmd_motor_ids_.size() != 2)
        {
            RCLCPP_FATAL(this->get_logger(), "rear_rmd_motor_ids must contain exactly 2 elements.");
            throw std::runtime_error("Invalid rear_rmd_motor_ids size");
        }

        front_left_rmd_id_ = static_cast<int>(front_rmd_motor_ids_[0]);
        front_right_rmd_id_ = static_cast<int>(front_rmd_motor_ids_[1]);

        front_left_vesc_id_ = static_cast<int>(front_vesc_motor_ids_[0]);
        front_right_vesc_id_ = static_cast<int>(front_vesc_motor_ids_[1]);

        rear_left_rmd_id_ = static_cast<int>(rear_rmd_motor_ids_[0]);
        rear_right_rmd_id_ = static_cast<int>(rear_rmd_motor_ids_[1]);

        drive_mode_ = DriveMode::DIFFERENTIAL;

        diff_ = std::make_unique<vehicle_kinematics::DifferentialDrive>(
            front_wheel_radius_, front_track_width_);

        two_ws_four_wd_ = std::make_unique<vehicle_kinematics::TwoWSFourWDDrive>(
            wheel_base_,
            front_track_width_,
            rear_track_width_,
            front_wheel_radius_,
            rear_wheel_radius_);

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
            "/docking_state",
            10,
            std::bind(&FrontbotControlNode::dockingStateCallback, this, std::placeholders::_1));

        front_rmd_command_pub_ =
            this->create_publisher<cartrider_rmd_sdk::msg::MotorCommandArray>("rmd_command", 10);

        front_vesc_command_pub_ =
            this->create_publisher<cartrider_vesc_sdk::msg::MotorCommandArray>("vesc_command", 10);

        rear_rmd_command_pub_ =
            this->create_publisher<cartrider_rmd_sdk::msg::MotorCommandArray>("/rear/rmd_command", 10);

        RCLCPP_INFO(
            this->get_logger(),
            "Frontbot Control Node Started. Input Mode: JOYSTICK, Drive Mode: %s",
            driveModeToString(drive_mode_).c_str());
    }

private:
    bool joy_mode_active_{true};
    DriveMode drive_mode_{DriveMode::DIFFERENTIAL};

    std::unique_ptr<vehicle_kinematics::DifferentialDrive> diff_;
    std::unique_ptr<vehicle_kinematics::TwoWSFourWDDrive> two_ws_four_wd_;

    double front_wheel_radius_{0.0};
    double rear_wheel_radius_{0.0};
    double front_track_width_{0.0};
    double rear_track_width_{0.0};
    double wheel_base_{0.0};

    std::vector<int64_t> front_rmd_motor_ids_;
    std::vector<int64_t> front_vesc_motor_ids_;
    std::vector<int64_t> rear_rmd_motor_ids_;

    int front_left_rmd_id_{0};
    int front_right_rmd_id_{0};
    int front_left_vesc_id_{0};
    int front_right_vesc_id_{0};
    int rear_left_rmd_id_{0};
    int rear_right_rmd_id_{0};

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_joy_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr joy_sig_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr docking_state_sub_;

    rclcpp::Publisher<cartrider_rmd_sdk::msg::MotorCommandArray>::SharedPtr front_rmd_command_pub_;
    rclcpp::Publisher<cartrider_vesc_sdk::msg::MotorCommandArray>::SharedPtr front_vesc_command_pub_;
    rclcpp::Publisher<cartrider_rmd_sdk::msg::MotorCommandArray>::SharedPtr rear_rmd_command_pub_;

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
        if (joy_mode_active_ == msg->data)
        {
            return;
        }

        joy_mode_active_ = msg->data;

        RCLCPP_INFO(
            this->get_logger(),
            "Control Input Mode Switched: [%s]",
            joy_mode_active_ ? "JOYSTICK" : "NAVIGATION");
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
        if (drive_mode_ == DriveMode::ACKERMANN)
        {
            publishAckermannCommand(msg, source);
        }
        else
        {
            publishDifferentialCommand(msg, source);
        }
    }

    void publishDifferentialCommand(
        const geometry_msgs::msg::Twist::SharedPtr msg,
        const std::string &source)
    {
        const auto output = diff_->compute(msg->linear.x, msg->angular.z);

        const double front_left_wheel_radps = output.left_w;
        const double front_right_wheel_radps = -output.right_w;

        cartrider_rmd_sdk::msg::MotorCommandArray front_rmd_cmd;
        cartrider_vesc_sdk::msg::MotorCommandArray front_vesc_cmd;

        cartrider_rmd_sdk::msg::MotorCommand front_left_drive_cmd;
        cartrider_rmd_sdk::msg::MotorCommand front_right_drive_cmd;

        cartrider_vesc_sdk::msg::MotorCommand front_left_yaw_cmd;
        cartrider_vesc_sdk::msg::MotorCommand front_right_yaw_cmd;

        front_left_drive_cmd.id = front_left_rmd_id_;
        front_right_drive_cmd.id = front_right_rmd_id_;

        front_left_yaw_cmd.id = front_left_vesc_id_;
        front_right_yaw_cmd.id = front_right_vesc_id_;

        front_left_drive_cmd.target = front_left_wheel_radps;
        front_right_drive_cmd.target = front_right_wheel_radps;

        front_left_yaw_cmd.target = 0.0;
        front_right_yaw_cmd.target = 0.0;

        front_rmd_cmd.commands.push_back(front_left_drive_cmd);
        front_rmd_cmd.commands.push_back(front_right_drive_cmd);

        front_vesc_cmd.commands.push_back(front_left_yaw_cmd);
        front_vesc_cmd.commands.push_back(front_right_yaw_cmd);

        front_rmd_command_pub_->publish(front_rmd_cmd);
        front_vesc_command_pub_->publish(front_vesc_cmd);

        RCLCPP_INFO(
            this->get_logger(),
            "[%s][DIFF] v=%.3f w=%.3f -> FL_w=%.3f FR_w=%.3f",
            source.c_str(),
            msg->linear.x,
            msg->angular.z,
            front_left_wheel_radps,
            front_right_wheel_radps);
    }

    void publishAckermannCommand(
        const geometry_msgs::msg::Twist::SharedPtr msg,
        const std::string &source)
    {
        const auto output = two_ws_four_wd_->compute(msg->linear.x, msg->angular.z);

        const double front_left_steer_rad = -output.front_left_steer;
        const double front_right_steer_rad = -output.front_right_steer;

        const double front_left_wheel_radps = output.front_left_w;
        const double front_right_wheel_radps = -output.front_right_w;

        const double rear_left_wheel_radps = -output.rear_left_w;
        const double rear_right_wheel_radps = output.rear_right_w;

        cartrider_rmd_sdk::msg::MotorCommandArray front_rmd_cmd;
        cartrider_vesc_sdk::msg::MotorCommandArray front_vesc_cmd;
        cartrider_rmd_sdk::msg::MotorCommandArray rear_rmd_cmd;

        cartrider_rmd_sdk::msg::MotorCommand front_left_drive_cmd;
        cartrider_rmd_sdk::msg::MotorCommand front_right_drive_cmd;
        cartrider_rmd_sdk::msg::MotorCommand rear_left_drive_cmd;
        cartrider_rmd_sdk::msg::MotorCommand rear_right_drive_cmd;

        cartrider_vesc_sdk::msg::MotorCommand front_left_yaw_cmd;
        cartrider_vesc_sdk::msg::MotorCommand front_right_yaw_cmd;

        front_left_drive_cmd.id = front_left_rmd_id_;
        front_right_drive_cmd.id = front_right_rmd_id_;

        rear_left_drive_cmd.id = rear_left_rmd_id_;
        rear_right_drive_cmd.id = rear_right_rmd_id_;

        front_left_yaw_cmd.id = front_left_vesc_id_;
        front_right_yaw_cmd.id = front_right_vesc_id_;

        front_left_drive_cmd.target = front_left_wheel_radps;
        front_right_drive_cmd.target = front_right_wheel_radps;

        rear_left_drive_cmd.target = rear_left_wheel_radps;
        rear_right_drive_cmd.target = rear_right_wheel_radps;

        front_left_yaw_cmd.target = front_left_steer_rad;
        front_right_yaw_cmd.target = front_right_steer_rad;

        front_rmd_cmd.commands.push_back(front_left_drive_cmd);
        front_rmd_cmd.commands.push_back(front_right_drive_cmd);

        front_vesc_cmd.commands.push_back(front_left_yaw_cmd);
        front_vesc_cmd.commands.push_back(front_right_yaw_cmd);

        rear_rmd_cmd.commands.push_back(rear_left_drive_cmd);
        rear_rmd_cmd.commands.push_back(rear_right_drive_cmd);

        front_rmd_command_pub_->publish(front_rmd_cmd);
        front_vesc_command_pub_->publish(front_vesc_cmd);
        rear_rmd_command_pub_->publish(rear_rmd_cmd);

        RCLCPP_INFO(
            this->get_logger(),
            "[%s][2WS4WD] v=%.3f w=%.3f -> FL_steer=%.3f FR_steer=%.3f FL_w=%.3f FR_w=%.3f RL_w=%.3f RR_w=%.3f",
            source.c_str(),
            msg->linear.x,
            msg->angular.z,
            front_left_steer_rad,
            front_right_steer_rad,
            front_left_wheel_radps,
            front_right_wheel_radps,
            rear_left_wheel_radps,
            rear_right_wheel_radps);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrontbotControlNode>());
    rclcpp::shutdown();
    return 0;
}