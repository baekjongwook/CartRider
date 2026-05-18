#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int16.hpp>

#include "cartrider_rmd_sdk/msg/motor_command_array.hpp"
#include "cartrider_vesc_sdk/msg/motor_command_array.hpp"

#include "cartrider_drive_controller/differential_drive.hpp"
#include "cartrider_drive_controller/ackermann_drive.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

class FrontbotControlNode : public rclcpp::Node
{
public:
    enum class DriveMode
    {
        DIFFERENTIAL,
        ACKERMANN
    };

    enum class DockingCommandMode
    {
        IDLE,
        DOCKING,
        UNDOCKING
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

        mightyzap_docking_speed_ =
            this->declare_parameter<int>("mightyzap_docking_speed", 500);
        mightyzap_docking_position_ =
            this->declare_parameter<int>("mightyzap_docking_position", 3000);
        mightyzap_release_position_ =
            this->declare_parameter<int>("mightyzap_release_position", 0);
        mightyzap_position_tolerance_ =
            this->declare_parameter<int>("mightyzap_position_tolerance", 20);

        mightyzap_docking_speed_ =
            std::clamp(mightyzap_docking_speed_, 0, 1023);
        mightyzap_docking_position_ =
            std::clamp(mightyzap_docking_position_, 0, 4095);
        mightyzap_release_position_ =
            std::clamp(mightyzap_release_position_, 0, 4095);
        mightyzap_position_tolerance_ =
            std::clamp(mightyzap_position_tolerance_, 0, 4095);

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

        front_left_rmd_id_ = static_cast<int>(front_rmd_motor_ids_[0]);
        front_right_rmd_id_ = static_cast<int>(front_rmd_motor_ids_[1]);

        front_left_vesc_id_ = static_cast<int>(front_vesc_motor_ids_[0]);
        front_right_vesc_id_ = static_cast<int>(front_vesc_motor_ids_[1]);

        drive_mode_ = DriveMode::DIFFERENTIAL;
        docking_command_mode_ = DockingCommandMode::IDLE;

        diff_ = std::make_unique<vehicle_kinematics::DifferentialDrive>(
            front_wheel_radius_,
            front_track_width_);

        two_ws_four_wd_ = std::make_unique<vehicle_kinematics::TwoWSFourWDDrive>(
            wheel_base_,
            front_track_width_,
            rear_track_width_,
            front_wheel_radius_,
            rear_wheel_radius_);

        front_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel",
            10,
            std::bind(&FrontbotControlNode::frontCmdCallback, this, std::placeholders::_1));

        multibot_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel",
            10,
            std::bind(&FrontbotControlNode::multibotCmdCallback, this, std::placeholders::_1));

        front_cmd_joy_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel_joy",
            10,
            std::bind(&FrontbotControlNode::frontCmdJoyCallback, this, std::placeholders::_1));

        multibot_cmd_joy_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_joy",
            10,
            std::bind(&FrontbotControlNode::multibotCmdJoyCallback, this, std::placeholders::_1));

        joy_sig_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "joy_control_sig",
            10,
            std::bind(&FrontbotControlNode::joySigCallback, this, std::placeholders::_1));

        docking_state_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/docking_state",
            10,
            std::bind(&FrontbotControlNode::dockingStateCallback, this, std::placeholders::_1));

        do_docking_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "do_docking",
            10,
            std::bind(&FrontbotControlNode::doDockingCallback, this, std::placeholders::_1));

        mightyzap_position_sub_ = this->create_subscription<std_msgs::msg::UInt16>(
            "present_position",
            10,
            std::bind(&FrontbotControlNode::mightyZapPositionCallback, this, std::placeholders::_1));

        front_rmd_command_pub_ =
            this->create_publisher<cartrider_rmd_sdk::msg::MotorCommandArray>(
                "rmd_command",
                10);

        front_vesc_command_pub_ =
            this->create_publisher<cartrider_vesc_sdk::msg::MotorCommandArray>(
                "vesc_command",
                10);

        mightyzap_force_enable_pub_ =
            this->create_publisher<std_msgs::msg::Bool>(
                "force_enable",
                10);

        mightyzap_goal_speed_pub_ =
            this->create_publisher<std_msgs::msg::UInt16>(
                "goal_speed",
                10);

        mightyzap_goal_position_pub_ =
            this->create_publisher<std_msgs::msg::UInt16>(
                "goal_position",
                10);

        docking_state_pub_ =
            this->create_publisher<std_msgs::msg::Bool>(
                "/docking_state",
                10);

        publishDockingState(false);

        RCLCPP_INFO(
            this->get_logger(),
            "Frontbot Control Node Started. Input Mode: JOYSTICK, Drive Mode: %s",
            driveModeToString(drive_mode_).c_str());

        RCLCPP_INFO(
            this->get_logger(),
            "mightyZAP Params: docking_speed=%d docking_position=%d release_position=%d tolerance=%d",
            mightyzap_docking_speed_,
            mightyzap_docking_position_,
            mightyzap_release_position_,
            mightyzap_position_tolerance_);
    }

private:
    bool joy_mode_active_{true};
    bool docking_state_{false};
    bool mightyzap_position_valid_{false};

    DriveMode drive_mode_{DriveMode::DIFFERENTIAL};
    DockingCommandMode docking_command_mode_{DockingCommandMode::IDLE};

    std::unique_ptr<vehicle_kinematics::DifferentialDrive> diff_;
    std::unique_ptr<vehicle_kinematics::TwoWSFourWDDrive> two_ws_four_wd_;

    double front_wheel_radius_{0.0};
    double rear_wheel_radius_{0.0};
    double front_track_width_{0.0};
    double rear_track_width_{0.0};
    double wheel_base_{0.0};

    std::vector<int64_t> front_rmd_motor_ids_;
    std::vector<int64_t> front_vesc_motor_ids_;

    int front_left_rmd_id_{0};
    int front_right_rmd_id_{0};
    int front_left_vesc_id_{0};
    int front_right_vesc_id_{0};

    int mightyzap_docking_speed_{500};
    int mightyzap_docking_position_{3000};
    int mightyzap_release_position_{0};
    int mightyzap_position_tolerance_{20};
    int mightyzap_active_target_position_{0};

    uint16_t mightyzap_present_position_{0};

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr front_cmd_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr multibot_cmd_sub_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr front_cmd_joy_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr multibot_cmd_joy_sub_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr joy_sig_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr docking_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr do_docking_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr mightyzap_position_sub_;

    rclcpp::Publisher<cartrider_rmd_sdk::msg::MotorCommandArray>::SharedPtr front_rmd_command_pub_;
    rclcpp::Publisher<cartrider_vesc_sdk::msg::MotorCommandArray>::SharedPtr front_vesc_command_pub_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mightyzap_force_enable_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr mightyzap_goal_speed_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr mightyzap_goal_position_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr docking_state_pub_;

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

    std::string dockingCommandModeToString(DockingCommandMode mode) const
    {
        switch (mode)
        {
        case DockingCommandMode::IDLE:
            return "IDLE";
        case DockingCommandMode::DOCKING:
            return "DOCKING";
        case DockingCommandMode::UNDOCKING:
            return "UNDOCKING";
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
        docking_state_ = msg->data;

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

    void doDockingCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data)
        {
            if (docking_state_ && docking_command_mode_ == DockingCommandMode::IDLE)
            {
                RCLCPP_INFO(this->get_logger(), "[DOCKING] Already docked. Ignoring do_docking=true.");
                return;
            }

            startDockingMotion(
                DockingCommandMode::DOCKING,
                mightyzap_docking_position_);

            RCLCPP_INFO(
                this->get_logger(),
                "[DOCKING] Docking started. target=%d speed=%d tolerance=%d",
                mightyzap_active_target_position_,
                mightyzap_docking_speed_,
                mightyzap_position_tolerance_);
        }
        else
        {
            if (!docking_state_ && docking_command_mode_ == DockingCommandMode::IDLE)
            {
                RCLCPP_INFO(this->get_logger(), "[DOCKING] Already released. Ignoring do_docking=false.");
                return;
            }

            startDockingMotion(
                DockingCommandMode::UNDOCKING,
                mightyzap_release_position_);

            RCLCPP_INFO(
                this->get_logger(),
                "[DOCKING] Undocking started. target=%d speed=%d tolerance=%d",
                mightyzap_active_target_position_,
                mightyzap_docking_speed_,
                mightyzap_position_tolerance_);
        }
    }

    void startDockingMotion(
        DockingCommandMode mode,
        int target_position)
    {
        docking_command_mode_ = mode;
        mightyzap_active_target_position_ =
            std::clamp(target_position, 0, 4095);

        publishFrontCommand(0.0, 0.0, 0.0, 0.0);
        publishMightyZapCommand(
            mightyzap_docking_speed_,
            mightyzap_active_target_position_,
            true);
    }

    void mightyZapPositionCallback(const std_msgs::msg::UInt16::SharedPtr msg)
    {
        mightyzap_present_position_ = msg->data;
        mightyzap_position_valid_ = true;

        if (docking_command_mode_ == DockingCommandMode::IDLE)
        {
            return;
        }

        const int error =
            std::abs(static_cast<int>(mightyzap_present_position_) - mightyzap_active_target_position_);

        if (error > mightyzap_position_tolerance_)
        {
            return;
        }

        const DockingCommandMode completed_mode = docking_command_mode_;
        docking_command_mode_ = DockingCommandMode::IDLE;

        if (completed_mode == DockingCommandMode::DOCKING)
        {
            docking_state_ = true;
            publishDockingState(true);

            RCLCPP_INFO(
                this->get_logger(),
                "[DOCKING] Docking complete. present=%u target=%d error=%d tolerance=%d",
                mightyzap_present_position_,
                mightyzap_active_target_position_,
                error,
                mightyzap_position_tolerance_);
        }
        else if (completed_mode == DockingCommandMode::UNDOCKING)
        {
            docking_state_ = false;
            publishDockingState(false);

            RCLCPP_INFO(
                this->get_logger(),
                "[DOCKING] Undocking complete. present=%u target=%d error=%d tolerance=%d",
                mightyzap_present_position_,
                mightyzap_active_target_position_,
                error,
                mightyzap_position_tolerance_);
        }
    }

    void publishMightyZapCommand(
        int speed,
        int position,
        bool force_enable)
    {
        std_msgs::msg::Bool force_msg;
        force_msg.data = force_enable;
        mightyzap_force_enable_pub_->publish(force_msg);

        std_msgs::msg::UInt16 speed_msg;
        speed_msg.data = static_cast<uint16_t>(
            std::clamp(speed, 0, 1023));
        mightyzap_goal_speed_pub_->publish(speed_msg);

        std_msgs::msg::UInt16 position_msg;
        position_msg.data = static_cast<uint16_t>(
            std::clamp(position, 0, 4095));
        mightyzap_goal_position_pub_->publish(position_msg);
    }

    void publishDockingState(bool state)
    {
        std_msgs::msg::Bool msg;
        msg.data = state;
        docking_state_pub_->publish(msg);
    }

    bool isDockingMotionActive() const
    {
        return docking_command_mode_ != DockingCommandMode::IDLE;
    }

    bool blockDriveCommandDuringDocking(const std::string &source)
    {
        if (!isDockingMotionActive())
        {
            return false;
        }

        publishFrontCommand(0.0, 0.0, 0.0, 0.0);

        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "[%s] Drive command blocked while docking command is active. mode=%s",
            source.c_str(),
            dockingCommandModeToString(docking_command_mode_).c_str());

        return true;
    }

    void frontCmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (blockDriveCommandDuringDocking("NAV_FRONT"))
        {
            return;
        }

        if (joy_mode_active_)
        {
            return;
        }

        if (drive_mode_ != DriveMode::DIFFERENTIAL)
        {
            return;
        }

        publishDifferentialCommand(msg, "NAV_FRONT");
    }

    void multibotCmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (blockDriveCommandDuringDocking("NAV_MULTIBOT"))
        {
            return;
        }

        if (joy_mode_active_)
        {
            return;
        }

        if (drive_mode_ != DriveMode::ACKERMANN)
        {
            return;
        }

        publishAckermannCommand(msg, "NAV_MULTIBOT");
    }

    void frontCmdJoyCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (blockDriveCommandDuringDocking("JOY_FRONT"))
        {
            return;
        }

        if (!joy_mode_active_)
        {
            return;
        }

        if (drive_mode_ != DriveMode::DIFFERENTIAL)
        {
            return;
        }

        publishDifferentialCommand(msg, "JOY_FRONT");
    }

    void multibotCmdJoyCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (blockDriveCommandDuringDocking("JOY_MULTIBOT"))
        {
            return;
        }

        if (!joy_mode_active_)
        {
            return;
        }

        if (drive_mode_ != DriveMode::ACKERMANN)
        {
            return;
        }

        publishAckermannCommand(msg, "JOY_MULTIBOT");
    }

    void publishDifferentialCommand(
        const geometry_msgs::msg::Twist::SharedPtr msg,
        const std::string &source)
    {
        const auto output = diff_->compute(msg->linear.x, msg->angular.z);

        const double front_left_wheel_radps = output.left_w;
        const double front_right_wheel_radps = -output.right_w;

        publishFrontCommand(
            front_left_wheel_radps,
            front_right_wheel_radps,
            0.0,
            0.0);

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

        publishFrontCommand(
            front_left_wheel_radps,
            front_right_wheel_radps,
            front_left_steer_rad,
            front_right_steer_rad);

        RCLCPP_INFO(
            this->get_logger(),
            "[%s][2WS4WD-FRONT] v=%.3f w=%.3f -> FL_steer=%.3f FR_steer=%.3f FL_w=%.3f FR_w=%.3f",
            source.c_str(),
            msg->linear.x,
            msg->angular.z,
            front_left_steer_rad,
            front_right_steer_rad,
            front_left_wheel_radps,
            front_right_wheel_radps);
    }

    void publishFrontCommand(
        double front_left_wheel_radps,
        double front_right_wheel_radps,
        double front_left_steer_rad,
        double front_right_steer_rad)
    {
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

        front_left_yaw_cmd.target = front_left_steer_rad;
        front_right_yaw_cmd.target = front_right_steer_rad;

        front_rmd_cmd.commands.push_back(front_left_drive_cmd);
        front_rmd_cmd.commands.push_back(front_right_drive_cmd);

        front_vesc_cmd.commands.push_back(front_left_yaw_cmd);
        front_vesc_cmd.commands.push_back(front_right_yaw_cmd);

        front_rmd_command_pub_->publish(front_rmd_cmd);
        front_vesc_command_pub_->publish(front_vesc_cmd);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrontbotControlNode>());
    rclcpp::shutdown();
    return 0;
}