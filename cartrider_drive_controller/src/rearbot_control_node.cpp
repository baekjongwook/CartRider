#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>

#include "cartrider_rmd_sdk/msg/motor_command_array.hpp"

#include "cartrider_drive_controller/differential_drive.hpp"
#include "cartrider_drive_controller/ackermann_drive.hpp"

class RearbotControlNode : public rclcpp::Node
{
public:
  enum class DriveMode
  {
    DIFFERENTIAL,
    ACKERMANN
  };

  RearbotControlNode()
      : Node("rearbot_control_node")
  {
    front_wheel_radius_ = this->declare_parameter<double>("front_wheel_radius");
    rear_wheel_radius_ = this->declare_parameter<double>("rear_wheel_radius");
    front_track_width_ = this->declare_parameter<double>("front_track_width");
    rear_track_width_ = this->declare_parameter<double>("rear_track_width");
    wheel_base_ = this->declare_parameter<double>("wheel_base");

    rear_rmd_motor_ids_ =
        this->declare_parameter<std::vector<int64_t>>("rear_rmd_motor_ids", std::vector<int64_t>{});

    if (rear_rmd_motor_ids_.size() != 2)
    {
      RCLCPP_FATAL(
          this->get_logger(),
          "Parameter 'rear_rmd_motor_ids' must contain exactly 2 elements. Got %zu",
          rear_rmd_motor_ids_.size());
      throw std::runtime_error("Invalid rear_rmd_motor_ids size");
    }

    rear_left_rmd_id_ = static_cast<int>(rear_rmd_motor_ids_[0]);
    rear_right_rmd_id_ = static_cast<int>(rear_rmd_motor_ids_[1]);

    drive_mode_ = DriveMode::DIFFERENTIAL;

    diff_ = std::make_unique<vehicle_kinematics::DifferentialDrive>(
        rear_wheel_radius_,
        rear_track_width_);

    two_ws_four_wd_ = std::make_unique<vehicle_kinematics::TwoWSFourWDDrive>(
        wheel_base_,
        front_track_width_,
        rear_track_width_,
        front_wheel_radius_,
        rear_wheel_radius_);

    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel",
        10,
        std::bind(&RearbotControlNode::cmdCallback, this, std::placeholders::_1));

    cmd_joy_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel_joy",
        10,
        std::bind(&RearbotControlNode::cmdJoyCallback, this, std::placeholders::_1));

    joy_sig_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "joy_control_sig",
        10,
        std::bind(&RearbotControlNode::joySigCallback, this, std::placeholders::_1));

    docking_state_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/docking_state",
        10,
        std::bind(&RearbotControlNode::dockingStateCallback, this, std::placeholders::_1));

    rear_rmd_command_pub_ =
        this->create_publisher<cartrider_rmd_sdk::msg::MotorCommandArray>(
            "rmd_command",
            10);

    RCLCPP_INFO(
        this->get_logger(),
        "Rearbot Control Node Started. Input Mode: JOYSTICK, Drive Mode: %s",
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

  std::vector<int64_t> rear_rmd_motor_ids_;

  int rear_left_rmd_id_{0};
  int rear_right_rmd_id_{0};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_joy_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr joy_sig_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr docking_state_sub_;

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

    const double rear_left_wheel_radps = -output.left_w;
    const double rear_right_wheel_radps = output.right_w;

    cartrider_rmd_sdk::msg::MotorCommandArray rear_rmd_cmd;

    cartrider_rmd_sdk::msg::MotorCommand rear_left_drive_cmd;
    cartrider_rmd_sdk::msg::MotorCommand rear_right_drive_cmd;

    rear_left_drive_cmd.id = rear_left_rmd_id_;
    rear_right_drive_cmd.id = rear_right_rmd_id_;

    rear_left_drive_cmd.target = rear_left_wheel_radps;
    rear_right_drive_cmd.target = rear_right_wheel_radps;

    rear_rmd_cmd.commands.push_back(rear_left_drive_cmd);
    rear_rmd_cmd.commands.push_back(rear_right_drive_cmd);

    rear_rmd_command_pub_->publish(rear_rmd_cmd);

    RCLCPP_INFO(
        this->get_logger(),
        "[%s][DIFF] v=%.3f w=%.3f -> RL_w=%.3f RR_w=%.3f",
        source.c_str(),
        msg->linear.x,
        msg->angular.z,
        rear_left_wheel_radps,
        rear_right_wheel_radps);
  }

  void publishAckermannCommand(
      const geometry_msgs::msg::Twist::SharedPtr msg,
      const std::string &source)
  {
    const auto output = two_ws_four_wd_->compute(msg->linear.x, msg->angular.z);

    const double rear_left_wheel_radps = -output.rear_left_w;
    const double rear_right_wheel_radps = output.rear_right_w;

    cartrider_rmd_sdk::msg::MotorCommandArray rear_rmd_cmd;

    cartrider_rmd_sdk::msg::MotorCommand rear_left_drive_cmd;
    cartrider_rmd_sdk::msg::MotorCommand rear_right_drive_cmd;

    rear_left_drive_cmd.id = rear_left_rmd_id_;
    rear_right_drive_cmd.id = rear_right_rmd_id_;

    rear_left_drive_cmd.target = rear_left_wheel_radps;
    rear_right_drive_cmd.target = rear_right_wheel_radps;

    rear_rmd_cmd.commands.push_back(rear_left_drive_cmd);
    rear_rmd_cmd.commands.push_back(rear_right_drive_cmd);

    rear_rmd_command_pub_->publish(rear_rmd_cmd);

    RCLCPP_INFO(
        this->get_logger(),
        "[%s][2WS4WD] v=%.3f w=%.3f -> RL_w=%.3f RR_w=%.3f",
        source.c_str(),
        msg->linear.x,
        msg->angular.z,
        rear_left_wheel_radps,
        rear_right_wheel_radps);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RearbotControlNode>());
  rclcpp::shutdown();
  return 0;
}