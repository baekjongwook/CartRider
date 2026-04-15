// Rearbot Control Node
// 2026.02.18 백종욱

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>

#include "cartrider_rmd_sdk/msg/motor_command_array.hpp"
#include "cartrider_drive_controller/differential_drive.hpp"

class RearbotControlNode : public rclcpp::Node
{
public:
  RearbotControlNode()
      : Node("rearbot_control_node")
  {
    r_ = this->declare_parameter<double>("wheel_radius");
    L_ = this->declare_parameter<double>("track_width");

    left_id_ = this->declare_parameter<int>("left_motor_id");
    right_id_ = this->declare_parameter<int>("right_motor_id");

    diff_ = std::make_unique<vehicle_kinematics::DifferentialDrive>(r_, L_);

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

    command_pub_ = this->create_publisher<cartrider_rmd_sdk::msg::MotorCommandArray>(
        "rmd_command",
        10);

    RCLCPP_INFO(this->get_logger(), "Rearbot Control Node Started. Default Mode: JOYSTICK");
  }

private:
  bool joy_mode_active_{true};

  std::unique_ptr<vehicle_kinematics::DifferentialDrive> diff_;

  double r_{0.0};
  double L_{0.0};

  int left_id_{0};
  int right_id_{0};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_joy_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr joy_sig_sub_;
  rclcpp::Publisher<cartrider_rmd_sdk::msg::MotorCommandArray>::SharedPtr command_pub_;

private:
  void joySigCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    joy_mode_active_ = msg->data;

    if (joy_mode_active_)
    {
      RCLCPP_INFO(this->get_logger(), "Control Mode Switched: [ JOYSTICK ]");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Control Mode Switched: [ NAVIGATION ]");
    }
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
    const auto output = diff_->compute(msg->linear.x, msg->angular.z);

    const double left_radps = -output.left_w;
    const double right_radps = output.right_w;

    RCLCPP_INFO(
        this->get_logger(),
        "[%s] cmd_vel: v=%.3f [m/s]  w=%.3f [rad/s]  ->  left=%.3f [rad/s]  right=%.3f [rad/s]",
        source.c_str(),
        msg->linear.x,
        msg->angular.z,
        left_radps,
        right_radps);

    cartrider_rmd_sdk::msg::MotorCommandArray cmd;

    cartrider_rmd_sdk::msg::MotorCommand left_cmd;
    left_cmd.id = left_id_;
    left_cmd.target = left_radps;

    cartrider_rmd_sdk::msg::MotorCommand right_cmd;
    right_cmd.id = right_id_;
    right_cmd.target = right_radps;

    cmd.commands.push_back(left_cmd);
    cmd.commands.push_back(right_cmd);

    command_pub_->publish(cmd);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RearbotControlNode>());
  rclcpp::shutdown();
  return 0;
}